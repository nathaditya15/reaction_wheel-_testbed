import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import time
import math

# Motor GPIO configuration
MOTOR_PINS = {
    "motor1": {"pwm_pin": 12, "dir_pin": 18},
    "motor2": {"pwm_pin": 13, "dir_pin": 19},
    "motor3": {"pwm_pin": 14, "dir_pin": 20}
}

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
for motor, pins in MOTOR_PINS.items():
    GPIO.setup(pins['pwm_pin'], GPIO.OUT)
    GPIO.setup(pins['dir_pin'], GPIO.OUT)
    MOTOR_PINS[motor]['pwm'] = GPIO.PWM(pins['pwm_pin'], 1000)  # Set PWM frequency to 1kHz
    MOTOR_PINS[motor]['pwm'].start(0)

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        current_time = time.time()
        delta_time = current_time - self.last_time

        if delta_time <= 0.0:
            delta_time = 1e-16  # Avoid division by zero

        # PID calculations
        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Update previous values
        self.prev_error = error
        self.last_time = current_time

        return output

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_controller_node')
        # Set up custom PID controllers for roll, pitch, and yaw
        self.pid_roll = PID(1.0, 0.1, 0.05)
        self.pid_pitch = PID(1.0, 0.1, 0.05)
        self.pid_yaw = PID(1.0, 0.1, 0.05)

        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(PoseStamped, 'imu_data', self.imu_callback, 10)

    def set_motor_speed(self, motor_name, speed):
        """Set speed and direction for a motor."""
        pwm = MOTOR_PINS[motor_name]['pwm']
        dir_pin = MOTOR_PINS[motor_name]['dir_pin']
        speed = max(-100, min(100, speed))  # Clamp speed between -100 and 100

        if speed >= 0:
            GPIO.output(dir_pin, GPIO.HIGH)
        else:
            GPIO.output(dir_pin, GPIO.LOW)
        pwm.ChangeDutyCycle(abs(speed))

        # Log the PWM duty cycle being sent to the motor
        self.get_logger().info(f"{motor_name} PWM: {abs(speed):.2f}% (Direction: {'Forward' if speed >= 0 else 'Reverse'})")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)  
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        """Callback function to receive IMU data and adjust motor speeds."""
        # Extract quaternion from PoseStamped message
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        # Calculate PID outputs
        roll_output = self.pid_roll.compute(0, roll)
        pitch_output = self.pid_pitch.compute(0, pitch)
        yaw_output = self.pid_yaw.compute(0, yaw)

        # Set motor speeds based on PID output and log each motor's PWM
        self.set_motor_speed("motor1", roll_output)
        self.set_motor_speed("motor2", pitch_output)
        self.set_motor_speed("motor3", yaw_output)

        # Log the motor outputs
        self.get_logger().info(f"Motor speeds - Roll: {roll_output:.2f}, Pitch: {pitch_output:.2f}, Yaw: {yaw_output:.2f}")

def main(args=None):
    rclpy.init(args=args)
    attitude_controller_node = AttitudeController()

    # Keep the node running
    rclpy.spin(attitude_controller_node)

    # Clean up on shutdown
    attitude_controller_node.destroy_node()
    rclpy.shutdown()
    for motor in MOTOR_PINS.values():
        motor['pwm'].stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

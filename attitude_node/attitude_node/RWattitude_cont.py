import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import time
import math
import pigpio

# Motor GPIO configuration
MOTOR_PINS = {
    "motor1": {"pwm_pin": 12, "dir_pin": 18},
    "motor2": {"pwm_pin": 13, "dir_pin": 19},
    "motor3": {"pwm_pin": 14, "dir_pin": 20},
    "motor4": {"pwm_pin": 15, "dir_pin": 21}
}

# Hall sensor pins
HALL_PINS = [2, 3, 4, 5]

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

        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        self.last_time = current_time

        return output

class AttitudeController(Node):
    def __init__(self):
        super().__init__('attitude_controller_node')
        self.pi = pigpio.pi()
        
        # Set up PID controllers
        self.pid_roll = PID(1.0, 0.1, 0.05)
        self.pid_pitch = PID(1.0, 0.1, 0.05)
        self.pid_yaw = PID(1.0, 0.1, 0.05)

        # Motor control variables
        self.rpm_measured = [0] * 4
        self.rpm_filtered = [0] * 4
        self.last_pulse_time = [0] * 4
        self.pulse_interval = [float('inf')] * 4

        # Initialize GPIO
        for motor, pins in MOTOR_PINS.items():
            self.pi.set_mode(pins['pwm_pin'], pigpio.OUTPUT)
            self.pi.set_mode(pins['dir_pin'], pigpio.OUTPUT)
            self.pi.set_PWM_frequency(pins['pwm_pin'], 1000)  # Set PWM frequency to 1kHz

        # Set up hall sensor interrupts
        for i, pin in enumerate(HALL_PINS):
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.callback(pin, pigpio.RISING_EDGE, self.pulse_counter)

        # Subscribe to the IMU data topic
        self.subscription = self.create_subscription(PoseStamped, 'imu_data', self.imu_callback, 10)

    def pulse_counter(self, gpio, level, tick):
        motor_index = HALL_PINS.index(gpio)
        current_time = time.time() * 1e6  # microseconds
        if self.last_pulse_time[motor_index] != 0:
            self.pulse_interval[motor_index] = current_time - self.last_pulse_time[motor_index]
        self.last_pulse_time[motor_index] = current_time

    def set_motor_speed(self, motor_name, speed):
        pins = MOTOR_PINS[motor_name]
        speed = max(-100, min(100, speed))  # Clamp speed between -100 and 100

        if speed >= 0:
            self.pi.write(pins['dir_pin'], 1)
        else:
            self.pi.write(pins['dir_pin'], 0)
        
        pwm_value = int(abs(speed) * 10000 / 100)  # Convert to 0-10000 range
        self.pi.set_PWM_dutycycle(pins['pwm_pin'], pwm_value)

        self.get_logger().info(f"{motor_name} PWM: {abs(speed):.2f}% (Direction: {'Forward' if speed >= 0 else 'Reverse'})")

    def quaternion_to_euler(self, x, y, z, w):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        roll_output = self.pid_roll.compute(0, roll)
        pitch_output = self.pid_pitch.compute(0, pitch)
        yaw_output = self.pid_yaw.compute(0, yaw)

        # Update motor speeds based on PID outputs
        self.set_motor_speed("motor1", roll_output)
        self.set_motor_speed("motor2", pitch_output)
        self.set_motor_speed("motor3", yaw_output)
        self.set_motor_speed("motor4", yaw_output)  # Assuming the fourth motor also contributes to yaw

        # Update RPM measurements
        for i in range(4):
            if self.pulse_interval[i] > 0 and self.pulse_interval[i] < 1e6:
                pulse_frequency = 1e6 / self.pulse_interval[i]
                self.rpm_measured[i] = (pulse_frequency * 60.0) / 7.0  # Assuming 7 pulses per revolution
            elif time.time() * 1e6 - self.last_pulse_time[i] > 500000:  # 500ms timeout
                self.rpm_measured[i] = 0

            # Simple low-pass filter for RPM
            self.rpm_filtered[i] = 0.9 * self.rpm_filtered[i] + 0.1 * self.rpm_measured[i]

        self.get_logger().info(f"Motor RPMs: {self.rpm_filtered}")

def main(args=None):
    rclpy.init(args=args)
    attitude_controller_node = AttitudeController()
    rclpy.spin(attitude_controller_node)
    attitude_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

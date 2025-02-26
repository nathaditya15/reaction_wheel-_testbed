import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import math

# Configuration constants (Adjust these based on your setup)
BNO055_I2C_ADDR = 0x28  # or 0x29 (check your sensor)

# Reaction Wheel Distribution
beta = 54.73 * (math.pi / 180)  # Radians
I = 0.0089  # Example moment of inertia for each wheel

# Define matrix A
A = np.array([
    [math.cos(beta), 0,          -math.cos(beta), 0],
    [0,          math.cos(beta), 0,          -math.cos(beta)],
    [math.sin(beta), math.sin(beta),  math.sin(beta),  math.sin(beta)]
])
A_pinv = np.linalg.pinv(A)  # Calculate pseudoinverse using NumPy

class DesiredOmegaCalculator(Node):
    def __init__(self):
        super().__init__('desired_omega_calculator')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'desired_omega', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize PD gains (Tuning parameters!)
        self.Kp_roll  = 1.0
        self.Kd_roll  = 0.05
        self.Kp_pitch = 1.0
        self.Kd_pitch = 0.05
        self.Kp_yaw   = 1.0
        self.Kd_yaw   = 0.0

        self.last_error_roll  = 0.0
        self.last_error_pitch = 0.0
        self.last_error_yaw   = 0.0

        # Simulated IMU Data (Replace with actual IMU readings)
        self.roll = 0.0   # Radians
        self.pitch = 0.0  # Radians
        self.yaw = 0.0    # Radians

    def timer_callback(self):
        """Calculates and publishes desired omega at a fixed rate."""

        # Simulate IMU data updates (replace with actual sensor readings)
        # In a real system, this would come from your IMU sensor
        self.roll  += 0.01  # Simulate change in roll
        self.pitch += 0.005 # Simulate change in pitch
        self.yaw   += 0.001  # Simulate change in yaw

        # Compute torques using PD control
        torque_roll  = self.compute_pd(self.roll, 0.0, self.Kp_roll, self.Kd_roll, self.last_error_roll)
        torque_pitch = self.compute_pd(self.pitch, 0.0, self.Kp_pitch, self.Kd_pitch, self.last_error_pitch)
        torque_yaw   = self.compute_pd(self.yaw, 0.0, self.Kp_yaw, self.Kd_yaw, self.last_error_yaw)

        # Create torque vector
        T = np.array([torque_roll, torque_pitch, torque_yaw])

        # Calculate desired angular velocity change (W_dot)
        W_dot = (1 / I) * np.dot(A_pinv, T)

        # Publish desired omega
        msg = Float64MultiArray()
        msg.data = W_dot.tolist()  # Convert numpy array to a list
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing desired omega: {msg.data}')

    def compute_pd(self, current_angle, setpoint, Kp, Kd, last_error):
        """Simple Proportional-Derivative (PD) controller."""
        error = setpoint - current_angle
        derivative = error - last_error
        output = Kp * error + Kd * derivative
        self.last_error = error  # Update last error for next iteration
        return output

def main(args=None):
    rclpy.init(args=args)
    desired_omega_calculator = DesiredOmegaCalculator()
    rclpy.spin(desired_omega_calculator)
    desired_omega_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import board
import busio
from adafruit_bno055 import BNO055_I2C

class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('attitude_node')

        # Create an I2C bus and initialize the sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = BNO055_I2C(i2c)
        time.sleep(1)  # Add delay to allow sensor initialization
        
        # **New Check for Sensor Connection**
        if self.sensor.quaternion is None:  
            self.get_logger().error("BNO055 sensor not detected. Please check the connection.")
            rclpy.shutdown()  # Shut down the node if sensor is not detected
            return

        # Create the publisher for the IMU data
        self.publisher = self.create_publisher(PoseStamped, 'imu_data', 10)

        # Timer to call the publish method periodically
        self.timer = self.create_timer(0.05, self.publish_data)  # publish every 0.4 seconds

    def publish_data(self):
        # Read quaternion from the sensor
        quat = self.sensor.quaternion
        if quat:
            qw, qx, qy, qz = quat
        else:
            # **New Warning if Sensor Data is Lost**
            self.get_logger().warning("BNO055 sensor lost connection!")
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0  # Default values if data is missing

        # Create a PoseStamped message
        pose_msg = PoseStamped()

        # Fill the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        # Set orientation as a quaternion
        self.get_logger().info(f"Publishing Quaternion - qw: {qw:.2f}, qx: {qx:.2f}, qy: {qy:.2f}, qz: {qz:.2f}")
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw 

        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the BNO055Publisher node
    bno055_publisher_node = BNO055Publisher()

    # Spin the node to keep it running
    rclpy.spin(bno055_publisher_node)

    # Clean up on shutdown
    bno055_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

class STLModelPublisher(Node):
    def __init__(self):
        super().__init__('stl_model_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.03, self.publish_marker)
        self.subscription = self.create_subscription(PoseStamped, 'imu_data', self.update_marker, 10)

        self.quatx = 0.0
        self.quaty = 0.0
        self.quatz = 0.0
        self.quatw = 1.0
    
    def update_marker(self, msg: PoseStamped):
        self.quatx = msg.pose.orientation.x
        self.quaty = msg.pose.orientation.y
        self.quatz = msg.pose.orientation.z
        self.quatw = msg.pose.orientation.w

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "satellite"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

    # def publish_marker(self):
    #     marker = Marker()
    #     marker.header.frame_id = "map"
    #     marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = "satellite"
    #     marker.id = 0
    #     marker.type = Marker.MESH_RESOURCE
    #     marker.action = Marker.ADD

        # Path to the STL file
        marker.mesh_resource = "file:///home/sdfcl/ros2_ws/src/attitude_control/meshes/ADITYA4.stl"

        # Position and orientation of the model
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # marker.pose.orientation.x = 0
        # marker.pose.orientation.y = 0
        # marker.pose.orientation.z = 0
        # marker.pose.orientation.w = 1

        marker.pose.orientation.x = self.quatx
        marker.pose.orientation.y = self.quaty
        marker.pose.orientation.z = self.quatz
        marker.pose.orientation.w = self.quatw

        # Scale of the model
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.publisher_.publish(marker)
        self.get_logger().info("Published STL model as a marker")

def main(args=None):
    rclpy.init(args=args)
    node = STLModelPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

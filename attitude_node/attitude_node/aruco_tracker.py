import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import cv2.aruco as aruco
import time
import math

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(PoseStamped, 'aruco_pose', 10)

        # Parameters
        self.id_to_find = self.declare_parameter('id_to_find', 24).get_parameter_value().integer_value
        self.marker_size = self.declare_parameter('marker_size', 180.0).get_parameter_value().double_value
        self.show_video = self.declare_parameter('show_video', True).get_parameter_value().bool_value
        self.camera_matrix = np.array([[426.3077, 0, 336.7072],
                                       [0, 423.5266, 237.9143],
                                       [0, 0, 1]], dtype=float)
        self.camera_distortion = np.array([-0.0489, -0.0279, 0.0, 0.0, 0.0])

        # ArUco setup
        self._aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self._parameters = aruco.DetectorParameters_create()

        # Video capture setup
        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture.")
            exit()

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        self._R_flip = np.zeros((3, 3), dtype=np.float32)
        self._R_flip[0, 0] = 1.0
        self._R_flip[1, 1] = -1.0
        self._R_flip[2, 2] = -1.0

        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.track)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return qx, qy, qz, qw

    def track(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn("Failed to capture frame from camera.")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict, parameters=self._parameters)

        if ids is not None and self.id_to_find in ids:
            # Estimate pose
            ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # Extract position
            x, y, z = tvec[0], tvec[1], tvec[2]

            # Calculate rotation matrix and Euler angles
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T
            roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip * R_tc)

            # Convert to quaternion
            qx, qy, qz, qw = self.quaternion_from_euler(roll_marker, pitch_marker, yaw_marker)

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'aruco_frame'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            # Publish the message
            self.publisher_.publish(pose_msg)

            self.get_logger().info(f"Published pose: x={x:.2f}, y={y:.2f}, z={z:.2f}")

            if self.show_video:
                # Draw marker and axes
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 50)
        else:
            self.get_logger().info("Marker not detected.")

        if self.show_video:
            cv2.imshow('Aruco Tracker', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()

    def _rotationMatrixToEulerAngles(self, R):
        """Convert rotation matrix to Euler angles."""
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

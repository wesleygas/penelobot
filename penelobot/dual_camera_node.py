import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_publisher')
        self.bridge = CvBridge()

        # Publishers
        self.left_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)

        # OpenCV VideoCapture
        self.left_cam = cv2.VideoCapture(0)
        self.right_cam = cv2.VideoCapture(1)

        if not self.left_cam.isOpened() or not self.right_cam.isOpened():
            self.get_logger().error("Could not open one or both video devices.")
            return

        # Timer
        self.timer = self.create_timer(1/30.0, self.publish_frames)

        # Calibration maps
        self.left_map1, self.left_map2 = None, None
        self.right_map1, self.right_map2 = None, None

        # Load calibration
        self.load_calibration_from_yaml()

    def load_calibration_from_yaml(self):
        pkg_share = self.get_package_share_directory("penelobot")
        calib_path = os.path.join(pkg_share, 'config', 'dual_camera_calibration.yaml')

        if not os.path.exists(calib_path):
            self.get_logger().warn(f"Calibration file not found: {calib_path}")
            return

        with open(calib_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        try:
            for side in ['left_camera', 'right_camera']:
                cam = calib_data[side]
                K = np.array(cam['camera_matrix']['data'], dtype=np.float32).reshape((3, 3))
                D = np.array(cam['distortion_coefficients']['data'], dtype=np.float32)
                width = cam.get('image_width', 640)
                height = cam.get('image_height', 480)
                size = (width, height)

                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, None, K, size, cv2.CV_16SC2
                )
                if side == 'left_camera':
                    self.left_map1, self.left_map2 = map1, map2
                else:
                    self.right_map1, self.right_map2 = map1, map2

            self.get_logger().info("Calibration maps loaded successfully from YAML.")

        except Exception as e:
            self.get_logger().error(f"Failed to load calibration from YAML: {e}")

    def get_package_share_directory(self, package_name):
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package_name)

    def publish_frames(self):
        ret_l, frame_l = self.left_cam.read()
        ret_r, frame_r = self.right_cam.read()

        if not ret_l or not ret_r:
            self.get_logger().warn("Failed to read from one of the cameras.")
            return

        # Rotate (example: 180°)
        frame_l = cv2.rotate(frame_l, cv2.ROTATE_90_CLOCKWISE)
        frame_r = cv2.rotate(frame_r, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Apply calibration maps
        if self.left_map1 is not None:
            frame_l = cv2.remap(frame_l, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        if self.right_map1 is not None:
            frame_r = cv2.remap(frame_r, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

        # Convert and publish
        msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
        msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')
        self.left_pub.publish(msg_l)
        self.right_pub.publish(msg_r)


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left_cam.release()
        node.right_cam.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

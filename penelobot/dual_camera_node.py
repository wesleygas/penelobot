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

        # Declare parameters
        self.declare_parameter('left_camera_index', 0)
        self.declare_parameter('right_camera_index', 1)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('left_rotation', 0)   # 0: no rotation
        self.declare_parameter('right_rotation', 0)
        self.declare_parameter('undistort_alpha', 0.0)  # 0.0: crop; 1.0: keep all


        # Get parameter values
        left_index = self.get_parameter('left_camera_index').get_parameter_value().integer_value
        right_index = self.get_parameter('right_camera_index').get_parameter_value().integer_value
        calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.left_rotation = self.get_parameter('left_rotation').get_parameter_value().integer_value
        self.right_rotation = self.get_parameter('right_rotation').get_parameter_value().integer_value
        self.undistort_alpha = self.get_parameter('undistort_alpha').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Publishers
        self.left_pub = self.create_publisher(Image, '/stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/stereo/right/image_raw', 10)

        # OpenCV VideoCapture
        self.left_cam = cv2.VideoCapture(left_index)
        self.right_cam = cv2.VideoCapture(right_index)

        if not self.left_cam.isOpened() or not self.right_cam.isOpened():
            self.get_logger().error("Could not open one or both video devices.")
            return

        self.left_cam.set(cv2.CAP_PROP_EXPOSURE, 40) 
        self.left_cam.set(cv2.CAP_PROP_EXPOSURE, 40) 

        # Load calibration maps (optional)
        self.left_map1, self.left_map2 = None, None
        self.right_map1, self.right_map2 = None, None
        if calib_file:
            self.load_calibration_from_yaml(calib_file)

        # Start publishing loop
        self.timer = self.create_timer(1/30.0, self.publish_frames)

        self.left_roi = None
        self.right_roi = None

    def load_calibration_from_yaml(self, calib_path):
        if not os.path.exists(calib_path):
            self.get_logger().warn(f"Calibration file not found: {calib_path}")
            return

        try:
            with open(calib_path, 'r') as f:
                calib_data = yaml.safe_load(f)

            for side in ['left_camera', 'right_camera']:
                cam = calib_data[side]
                K = np.array(cam['camera_matrix']['data'], dtype=np.float32).reshape((3, 3))
                D = np.array(cam['distortion_coefficients']['data'], dtype=np.float32)
                width = cam.get('image_width', 480)
                height = cam.get('image_height', 640)
                size = (width, height)

                # Adjust camera matrix to minimize cropping or preserve FOV
                new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, size, self.undistort_alpha)

                map1, map2 = cv2.initUndistortRectifyMap(
                    K, D, None, new_K, size, cv2.CV_16SC2
                )

                if side == 'left_camera':
                    self.left_map1, self.left_map2 = map1, map2
                    self.left_roi = roi
                else:
                    self.right_map1, self.right_map2 = map1, map2
                    self.right_roi = roi

            self.get_logger().info(f"Calibration maps loaded with alpha={self.undistort_alpha}")

        except Exception as e:
            self.get_logger().error(f"Failed to load calibration from YAML: {e}")


    def apply_rotation(self, frame, rotation_code):
        if rotation_code == 1:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif rotation_code == 2:
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif rotation_code == 3:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame  # No rotation

    def publish_frames(self):
        ret_l, frame_l = self.left_cam.read()
        ret_r, frame_r = self.right_cam.read()

        if not ret_l or not ret_r:
            self.get_logger().warn("Failed to read from one of the cameras.")
            return

        # Apply rotation
        # self.get_logger().info(f"left frame: {frame_l.shape[:2]}")
        frame_l = self.apply_rotation(frame_l, self.left_rotation)
        frame_r = self.apply_rotation(frame_r, self.right_rotation)
        # self.get_logger().info(f"left frame after rotation: {frame_l.shape[:2]}")

        # Apply calibration remapping if available
        if self.left_map1 is not None:
            frame_l = cv2.remap(frame_l, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            if self.left_roi is not None:
                x, y, w, h = self.left_roi
                frame_l = frame_l[y:y+h, x:x+w]
                # frame_l = cv2.resize(frame_l, (640, 480))
        if self.right_map1 is not None:
            frame_r = cv2.remap(frame_r, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
            if self.right_roi is not None:
                x, y, w, h = self.right_roi
                frame_r = frame_r[y:y+h, x:x+w]
                # frame_r = cv2.resize(frame_r, (640, 480))

        # Convert to ROS Image messages and publish
        # Convert and set metadata correctly
        msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
        msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')

        # self.get_logger().info(f"Publishing left frame: {frame_l.shape[:2]}")
        msg_l.height, msg_l.width = frame_l.shape[:2]
        msg_r.height, msg_r.width = frame_r.shape[:2]

        msg_l.step = frame_l.shape[1] * frame_l.shape[2] if frame_l.ndim == 3 else frame_l.shape[1]
        msg_r.step = frame_r.shape[1] * frame_r.shape[2] if frame_r.ndim == 3 else frame_r.shape[1]

        msg_l.header.frame_id = "camera_left"
        msg_r.header.frame_id = "camera_right"
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to the calibration file
    calib_file_path = PathJoinSubstitution([
        FindPackageShare("penelobot"),
        "config",
        "dual_camera_calibration.yaml"
    ])

    # Node definition
    dual_camera_node = Node(
        package='penelobot',  # Replace with your actual package name
        executable='dual_camera_node',  # Must match your console script entry point
        name='dual_camera_node',
        parameters=[{
            'left_camera_index': 0,
            'right_camera_index': 1,
            'calibration_file': calib_file_path,
            'left_rotation': 1,   # 90° clockwise
            'right_rotation': 3,   # 90° counter-clockwise
            'undistort_alpha': 0.0  # 0.0: crop; 1.0: keep all
        }],
        output='screen'
    )

    return LaunchDescription([
        dual_camera_node
    ])

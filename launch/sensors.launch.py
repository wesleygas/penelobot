import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


# from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_params_file = PathJoinSubstitution([FindPackageShare("penelobot"), 'config','pseye_camera_params.yaml'])
    launch_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='pseye_video_source',
        parameters=[camera_params_file],
        remappings=[
            ("/image_raw", "/camera/image_raw"),
            ("/image_raw/compressed", "/camera/compressed"),
            ("/camera_info", "/camera/camera_info")
        ]
    )

    lidar_params_file = PathJoinSubstitution([FindPackageShare("penelobot"), 'config','ydlidar.yaml'])
    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params_file],
        namespace='/',
    )

    # Path to the parameters file
    tof_params_file = PathJoinSubstitution([FindPackageShare("sipeed_tof_ms_a010"), 'config','tof_params.yaml'])
    tof_node = LaunchDescription([
        Node(
            package='sipeed_tof_ms_a010',
            executable='sipeed_tof_node',
            name='sipeed_tof_node',
            output='screen',
            emulate_tty=True,
            parameters=[tof_params_file]
        )
    ])
    
    return LaunchDescription([
        launch_camera,
        lidar_node,
        tof_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


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

    # transport_compress = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='image_compressing_node',
    #     remappings=[
    #         ("in","/camera/image_raw"),
    #         ("out","/camera/image_raw/compressed")
    #     ],
    #     arguments=['raw', 'compressed']
    # )

    return LaunchDescription([
        launch_camera,
        # transport_compress
    ])

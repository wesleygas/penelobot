from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    camera_params_file = PathJoinSubstitution([FindPackageShare("penelobot"), 'config','dual_pseye_camera_params.yaml'])
    launch_camera_0 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='pseye_video_0',
        parameters=[camera_params_file],
        remappings=[("/image_raw", "/stereo/left/image_raw")]
    )
    launch_camera_1 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='pseye_video_1',
        parameters=[camera_params_file],
        remappings=[("/image_raw", "/stereo/right/image_raw")]
    )

    # transport_compress = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='image_compressing_node',
    #     remappings=[
    #         ("in","/stereo/left/image_raw"),
    #         ("out","/camera/image_raw/compressed")
    #     ],
    #     arguments=['raw', 'compressed']
    # )

    return LaunchDescription([
        launch_camera_0,
        launch_camera_1
        # transport_compress
    ])

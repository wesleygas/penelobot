from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_camera = Node(
        package='ros_deep_learning',
        executable='video_source',
        name='jetson_video_source',
        parameters=[{
            "resource":"csi://0",
            "width":0,
            "height":0,
            "rtsp_latency":0,
            "loop":0
        }],
        remappings=[
            ("/video_source/raw", "/camera/image_raw"),
        ]
    )

    transport_compress = Node(
        package='image_transport',
        executable='republish',
        name='image_compressing_node',
        remappings=[
            ("in","/camera/image_raw"),
            ("out","/camera/image_raw/compressed")
        ],
        arguments=['raw', 'compressed']
    )

    return LaunchDescription([
        launch_camera,
        transport_compress
    ])

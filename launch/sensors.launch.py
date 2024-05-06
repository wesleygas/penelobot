import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

# from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    lidar_params_declare = DeclareLaunchArgument(
        'lidar_params_file',
        default_value=os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'),
            'params', 'ydlidar.yaml'
            ),
        description='FPath to the ROS2 parameters file to use.'
    )
    lidar_parameter_file = LaunchConfiguration('lidar_params_file')
    lidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_parameter_file],
        namespace='/',
    )
    # lidar_tf2_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_laser',
    #     arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
    # )
    # imu_tf2_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_laser',
    #     arguments=['0', '0', '0.0','0', '0', '0', '1','base_link','map'],
    # )

    # imu_params_declare = DeclareLaunchArgument(
    #     'imu_params_file',
    #     default_value=os.path.join(
    #         get_package_share_directory('mpu6050driver'), 'params', 'mpu6050.yaml'),
    #     description='Path to the ROS2 parameters file to use.')
    # imu_parameter_file = LaunchConfiguration('imu_params_file')
    # mpu6050driver_node = Node(
    #     package='mpu6050driver',
    #     executable='mpu6050driver',
    #     name='mpu6050driver_node',
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[imu_parameter_file],
    #     remappings=[
    #         ("/imu","/imu/data_raw")
    #     ]
    # )
    # complementary_filter = Node(
    #     package='imu_complementary_filter',
    #     executable='complementary_filter_node',
    #     name='complementary_filter_gain_node',
    #     output='screen',
    #     parameters=[
    #         {'do_bias_estimation': True},
    #         {'do_adaptive_gain': True},
    #         {'use_mag': False},
    #         {'gain_acc': 0.01},
    #         {'gain_mag': 0.01},
    #     ],
    # )
    return LaunchDescription([
        launch_camera,
        transport_compress,
        lidar_params_declare,
        lidar_node,
        # imu_tf2_node,
        # lidar_tf2_node,
        # imu_params_declare,
        # mpu6050driver_node,
        # complementary_filter
        #rviz2_node
    ])

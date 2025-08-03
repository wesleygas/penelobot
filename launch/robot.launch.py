import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='penelobot'

    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='false',
        description='Set to true to launch the robot sensors'
    )
    launch_sensors = LaunchConfiguration('launch_sensors')

    use_c_controller = DeclareLaunchArgument(
        'use_c_controller',
        default_value='true',
        description='Set to false to use the python controller'
    )
    use_c_controller = LaunchConfiguration('use_c_controller')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution(
                        [FindPackageShare(package_name),'launch','rsp.launch.py']
                    )
                ]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items() # Set to false
    )
    
    custom_controller_node = Node(
        package=package_name,
        executable='custom_controller', 
        name='custom_controller',
        output='screen',
        parameters=[{
            'device_port': '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:83:BA:2C-if00',
            'baud_rate': 230400,
            'loop_rate': 60.0,
            'wheel_separation': 0.204,
            'wheel_radius': 0.033,
            'enc_counts_per_rev': 1975.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'motor_acceleration': 9000.0,
            'battery_reading_period': 10.0
        }],
        condition=UnlessCondition(use_c_controller)
    )

    custom_c_controller_node = Node(
        package="serial_diffdrive_controller_node",
        executable='serial_diffdrive_controller_node', 
        name='diffdrive_controller',
        output='screen',
        parameters=[{
            'device_port': '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_64:E8:33:83:BA:2C-if00',
            'baud_rate': 230400,
            'loop_rate': 60.0,
            'wheel_separation': 0.204,
            'wheel_radius': 0.033,
            'enc_counts_per_rev': 1975.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'motor_acceleration': 9000.0,
            'battery_reading_period': 10.0
        }],
        condition=IfCondition(use_c_controller)
    )

    twist_mux_params = PathJoinSubstitution([FindPackageShare(package_name),'config','twist_mux.yaml'])
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/cmd_vel_out')]
        )

    sensors_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), 'launch', 'sensors.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        condition=IfCondition(launch_sensors)
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        custom_controller_node,
        custom_c_controller_node,
        launch_sensors_arg,
        sensors_launch_include
    ])
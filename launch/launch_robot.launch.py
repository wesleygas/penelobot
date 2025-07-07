import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution



def generate_launch_description():

    package_name='penelobot'

    # Declare the launch argument
    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='false',
        description='Set to true to launch the robot sensors'
    )
    launch_sensors = LaunchConfiguration('launch_sensors')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution(
                        [FindPackageShare(package_name),'launch','rsp.launch.py']
                    )
                ]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )
    
    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description", "robot.urdf.xacro"]
            ),
            ' use_ros2_control:=', 'true', ' sim_mode:=', 'false'
        ]
    )
    
    controller_params_file = PathJoinSubstitution([FindPackageShare(package_name), 'config','my_controllers.yaml'])
    # controller_params_file = PathJoinSubstitution([FindPackageShare(package_name), "config", "diffbot_controllers.yaml"])
    
    
    # print("robot_description",robot_description)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    twist_mux_params = PathJoinSubstitution([FindPackageShare(package_name),'config','twist_mux.yaml'])
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    sensors_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), 'launch', 'sensors.launch.py']
            )
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
        # This action will only be executed if the 'launch_sensors' argument is 'true'
        condition=IfCondition(launch_sensors)
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_manager,
        twist_mux,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        launch_sensors_arg,
        sensors_launch_include
    ])
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution



def generate_launch_description():

    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='penelobot'

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
    delayed_controller_manager = TimerAction(period=1.0, actions=[controller_manager])

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



    # Launch them all!
    return LaunchDescription([
        rsp,
        controller_manager,
        # delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the node for the Sipeed TOF MS-A010 3D camera.
    
    This launch file is equivalent to running the following command:
    ros2 run sipeed_tof_ms_a010 publisher --ros-args -p device:="/dev/ttyUSB0"
    """
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        arguments=["0", "0", "0", "0", "-1.5707", "0", "map", "tof"]
    )

    # Node for the Sipeed TOF 3D Camera
    tof_camera_node = Node(
        package='sipeed_tof_ms_a010',
        executable='publisher',
        name='tof_3d_camera_node', 
        parameters=[{
            'device': '/dev/ttyUSB0'
        }],
        remappings=[
            ("/cloud", "/camera/depth/points")
        ]
    )

    return LaunchDescription([
        tof_camera_node,
        static_tf
    ])
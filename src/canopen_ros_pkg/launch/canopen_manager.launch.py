import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    canopen_manager_path = get_package_share_directory('canopen_ros_pkg')
    
    can_interface = LaunchConfiguration('can_interface')
    heartbeat_interval = LaunchConfiguration('heartbeat_interval')
    can_bitrate = LaunchConfiguration('can_bitrate')
    can_txqueue_len = LaunchConfiguration('can_txqueue_len')
    motors_info_path = LaunchConfiguration('motors_info_path')
    
    canopen_manager_node = Node(
        package='canopen_ros_pkg',
        executable='canopen_manager',
        name='canopen_manager',
        parameters=[
            {'can_interface': can_interface},
            {'heartbeat_interval': heartbeat_interval},
            {'can_bitrate': can_bitrate},
            {'can_txqueue_len': can_txqueue_len},
            {'motors_info_path': motors_info_path}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'heartbeat_interval',
            default_value='1000',
            description='Heartbeat interval'
        ),
        DeclareLaunchArgument(
            'can_bitrate',
            default_value='1000',
            description='CAN bitrate'
        ),
        DeclareLaunchArgument(
            'can_txqueue_len',
            default_value='1000',
            description='CAN TX queue length'
        ),
        DeclareLaunchArgument(
            'motors_info_path',
            default_value=os.path.join(canopen_manager_path, 'config', 'canopen_info.json'),
            description='Motors info path'
        ),
        canopen_manager_node
    ])

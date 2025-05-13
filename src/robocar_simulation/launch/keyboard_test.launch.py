from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robocar_simulation')
    
    # Launch Gazebo with the maze (using maze.sdf only)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_share, 'worlds', 'maze.sdf')],
        output='screen'
    )
    
    # Launch our application nodes
    sensor_node = Node(
        package='robocar_simulation',
        executable='sensor_node',
        name='sensor_node',
        output='screen'
    )
    
    logger_node = Node(
        package='robocar_simulation',
        executable='logger_node',
        name='logger_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        sensor_node,
        logger_node,
        # The keyboard_control node is launched separately in a terminal
        # because it needs to capture keyboard input directly
    ])
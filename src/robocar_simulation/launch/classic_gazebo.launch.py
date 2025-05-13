from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robocar_simulation')
    
    # Process XACRO file to get URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'robocar.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Launch Gazebo with empty world (classic Gazebo instead of Ignition)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             os.path.join(pkg_share, 'worlds', 'maze.sdf')],
        output='screen'
    )
    
    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robocar',
            '-x', '1.0',
            '-y', '0.1',
            '-z', '0.05',
        ],
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
    
    command_monitor = Node(
        package='robocar_simulation',
        executable='command_monitor',
        name='command_monitor',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        sensor_node,
        logger_node,
        command_monitor
    ])
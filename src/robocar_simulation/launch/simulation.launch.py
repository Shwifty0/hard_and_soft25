from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robocar_simulation')
    
    # Launch Gazebo Harmonium
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_share, 'worlds', 'maze.sdf')],
        output='screen'
    )
    
    # Process XACRO file to get URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'robocar.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Spawn robot in Gazebo using ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robocar',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '0.1',
            '-z', '0.05',
        ],
        output='screen'
    )
    
    # Bridge to connect ROS2 topics with Gazebo
    # This bridges cmd_vel from ROS2 to Gazebo's model-specific topic
    # UPDATED: Using correct Harmonium bridge syntax with directionality
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # ROS to Gazebo bridge (cmd_vel → /model/robocar/cmd_vel)
            'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist@/model/robocar/cmd_vel',
            # Gazebo to ROS bridge (odom from Gazebo → ROS)
            '/model/robocar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
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
        bridge_cmd_vel,
        sensor_node,
        logger_node,
        command_monitor
    ])
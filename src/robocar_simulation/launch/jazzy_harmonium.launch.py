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
    
    # IMPORTANT: For Gazebo Harmonium/Garden in ROS 2 Jazzy, we need to use specific bridge configurations
    # This bridges from ROS 2 cmd_vel to Gazebo's model-specific topic
    bridge_ros_to_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_to_gz',
        arguments=[
            # From ROS 2 cmd_vel to Gazebo model/robocar/cmd_vel
            'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist@/model/robocar/cmd_vel',
        ],
        output='screen'
    )
    
    # This bridges from Gazebo to ROS 2 for sensor data and other feedback
    bridge_gz_to_ros = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_to_ros',
        arguments=[
            # From Gazebo to ROS 2 for joint states and other feedback
            '/model/robocar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/maze/model/robocar/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
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
        bridge_ros_to_gz,
        bridge_gz_to_ros,
        sensor_node,
        logger_node,
        command_monitor
    ])
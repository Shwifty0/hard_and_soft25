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
    # This bridges cmd_vel from ROS2 to Gazebo (modified to match keyboard_control's topic)
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )
    
    # This bridges joint states from Gazebo to ROS2
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'],
        output='screen'
    )
    
    # Launch our application nodes
    sensor_node = Node(
        package='robocar_simulation',
        executable='sensor_node',
        name='sensor_node',
        output='screen'
    )
    
    controller_node = Node(
        package='robocar_simulation',
        executable='controller_node',
        name='controller_node',
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
        robot_state_publisher,
        spawn_entity,
        bridge_cmd_vel,
        bridge_joint_states,
        sensor_node,
        controller_node,
        logger_node
    ])
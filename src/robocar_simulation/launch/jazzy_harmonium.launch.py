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
    urdf_file = os.path.join(pkg_share, 'urdf', 'robocar_harmonium_basic.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()
    
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
    
    # Bridge ROS 2 to Gazebo (for cmd_vel)
    bridge_ros_to_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_to_gz',
        arguments=[
            # Bridge both with and without leading slash for compatibility
            'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist@/model/robocar/cmd_vel',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist@/model/robocar/cmd_vel',
        ],
        output='screen'
    )
    
    # Bridge Gazebo to ROS 2 (for sensor data)
    bridge_gz_to_ros = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_to_ros',
        arguments=[
            '/model/robocar/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/maze/model/robocar/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        output='screen'
    )
    
    # Command monitor to debug commands
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
        command_monitor
    ])
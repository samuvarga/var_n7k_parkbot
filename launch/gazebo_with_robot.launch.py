import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    # Setup project paths
    pkg_var_n7k_parkbot = get_package_share_directory('var_n7k_parkbot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # SDF robot file path
    sdf_file = os.path.join(pkg_var_n7k_parkbot, 'robot_description', 'tb3_plain.sdf')

    # Gazebo world indítása
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_var_n7k_parkbot,
            'world',
            'parking_lot.sdf'
        ])}.items(),
    )

    # Spawn TurtleBot3
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-name', 'turtlebot3',
            '-file', sdf_file,
            '-x', '0', '-y', '0', '-z', '0.2'
        ]
    )

    # Bridge-ek
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        arguments=['/model/turtlebot3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        arguments=['/model/turtlebot3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry']
    )

    points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='points_bridge',
        output='screen',
        arguments=['/model/turtlebot3/scan@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked']
    )

    # Parkolási logika node
    parking_logic_node = Node(
        package='var_n7k_parkbot',
        executable='parking_logic.py',
        name='parking_logic_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        cmd_vel_bridge,
        odom_bridge,
        points_bridge,
        parking_logic_node
    ])
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Indítsd az Ignition Gazebo-t a world-del
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', 
                '/home/ajr/ros2_ws/src/var_n7k_parkbot/world/parking_lot.sdf',  
                '--verbose'
            ],
            output='screen'
        ),
        # Spawn TurtleBot3 Ignition Gazebo-ba
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_turtlebot3',
            output='screen',
            arguments=[
                '-name', 'turtlebot3',
                '-file', '/home/ajr/ros2_ws/src/var_n7k_parkbot/robot_description/tb3_plain.urdf',
                '-x', '0', '-y', '0', '-z', '0.1'
            ]
        ),
        # Bridge a /cmd_vel topicra (ROS <-> Ignition)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/model/turtlebot3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            output='screen',
            arguments=['/model/turtlebot3/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry']
        ),
        # Bridge a PointCloud2 topicra (ROS <-> Ignition)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='points_bridge',
            output='screen',
            arguments=['/model/turtlebot3/scan@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked']
        ),
        # Parkolási logika node
        Node(
            package='var_n7k_parkbot',
            executable='parking_logic_node',
            name='parking_logic_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
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
                '-file', '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf',
                '-x', '0', '-y', '0', '-z', '0.1'
            ]
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
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '/home/ajr/ros2_ws/src/var_n7k_parkbot/world/parking_lot.world'
            ],
            output='screen'
        ),
        Node(
            package='turtlebot3_description',
            executable='spawn_turtlebot3',
            name='spawn_turtlebot3',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-x', '0', '-y', '0', '-z', '0', '-Y', '0']
        ),
        Node(
            package='parking_logic_node',
            executable='parking_logic_node',
            name='parking_logic_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
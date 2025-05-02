from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            remappings=[
                ('cmd_vel', 'turtlebot3/cmd_vel')
            ]
        ),
        Node(
            package='rclcpp',
            executable='node_name',  # Replace with the actual node name if needed
            name='your_node_name',    # Replace with the actual node name if needed
            output='screen'
        )
    ])
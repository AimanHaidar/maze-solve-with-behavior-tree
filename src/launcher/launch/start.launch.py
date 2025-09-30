import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node
        Node(
            package='maze_graphics',
            executable='visualize_mazing',
            name='maze_gui',
            output='screen',
        ),
        # Second node
        Node(
            package='maze_solver',
            executable='one_hand_maze_solver',
            name='one_hand_maze_solver',
            output='screen',
        ),
    ])



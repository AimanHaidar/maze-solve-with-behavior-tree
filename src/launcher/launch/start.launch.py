import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_config = os.path.join(get_package_share_directory("launcher"),"config", "maze_config.yaml")
    maze_size_arg = DeclareLaunchArgument(
        'maze_size', default_value='big',
            description='put small,middle,big for maze size'
    )
    return LaunchDescription([
        
        # First node
        Node(
            package='maze_graphics',
            executable='visualize_mazing',
            name='maze_visualizer',
            output='screen',
            parameters=[
                {'maze_size' : LaunchConfiguration('maze_size')}
            ]
        ),
        # Second node
        Node(
            package='maze_solver',
            executable='one_hand_maze_solver',
            name='hand_maze_solver',
            output='screen',
            parameters=[param_config]
        ),
    ])



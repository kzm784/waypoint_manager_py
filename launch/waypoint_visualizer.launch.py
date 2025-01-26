from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoint_visualizer_config = os.path.join(
        get_package_share_directory('waypoint_manager_py'),
        'config',
        'visualizer_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager_py',
            executable='waypoint_visualizer',
            name='waypoint_visualizer_node',
            output='screen',
            parameters=[waypoint_visualizer_config]
        ),
    ])

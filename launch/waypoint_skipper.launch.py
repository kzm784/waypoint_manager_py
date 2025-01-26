from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    waypoint_skipper_config = os.path.join(
        get_package_share_directory('waypoint_manager_py'),
        'config',
        'skipper_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='waypoint_manager_py',
            executable='waypoint_skipper',
            name='waypoint_skipper_node',
            output='screen',
            remappings=[('/current_pose', '/current_pose'),
                        ('/scan', '/scan')
            ],
            parameters=[waypoint_skipper_config]
        ),
    ])

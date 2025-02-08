from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to the waypoint CSV
    navigation_data_dir = os.getenv('NAVIGATION_DATA_DIR')
    navigation_data_name = os.getenv('NAVIGATION_DATA_NAME')
    waypoints_csv_path = os.path.join(
        navigation_data_dir,
        navigation_data_name,
        f"{navigation_data_name}_wp.csv"
    )
    # waypoints_csv_path = '/home/kazuma/dev_ws/src/waypoint_manager/waypoints/test.csv'

    # Set the path to the Configuration file
    config_file_path = os.path.join(
        get_package_share_directory('waypoint_manager_py'),
        'config',
        'manager_config.yaml'
    )

    return LaunchDescription([
        # Waypoint manager node
        Node(
            package='waypoint_manager_py',
            executable='waypoint_manager',
            name='waypoint_manager_node',
            output='screen',
            remappings=[('/current_pose', '/pcl_pose')],
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),

        # Waypoint visualizer node
        Node(
            package='waypoint_manager_py',
            executable='waypoint_visualizer',
            name='waypoint_visualizer_node',
            output='screen',
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),

        # Waypoint skipper node
        Node(
            package='waypoint_manager_py',
            executable='waypoint_skipper',
            name='waypoint_skipper_node',
            output='screen',
            remappings=[('/current_pose', '/current_pose'),
                        ('/scan', '/scan')],
            parameters=[config_file_path, {
                'waypoints_csv': waypoints_csv_path,
            }]
        ),
    ])

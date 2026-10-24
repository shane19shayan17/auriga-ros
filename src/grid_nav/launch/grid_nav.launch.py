import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('grid_nav')
    params = os.path.join(share, 'config', 'planner_params.yaml')
    default_map = os.path.join(share, 'maps', 'map_example.txt')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map),

        # 1) the map (latched topic /map)
        Node(package='grid_nav', executable='map_publisher', output='screen',
             parameters=[{'map_file': LaunchConfiguration('map_file'),
                          'resolution': 1.0}]),

        # 2) costmap wants a TF between global_frame and robot_base_frame
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']),

        # 3) the planner (+ internal global_costmap)
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params]),

        # 4) lifecycle nodes start "unconfigured"; this brings them to ACTIVE
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planning', output='screen',
             parameters=[params]),
    ])

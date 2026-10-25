import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'navfn_params.yaml')
    
    print(params)

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{'autostart': True,
                    'node_names': ['planner_server']}],
    )

    return LaunchDescription([
        planner,
        lifecycle
    ])
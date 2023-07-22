import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'pure_pursuit.yaml'
    )
    
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pp_node',
        parameters=[config],
        name='pure_pursuit_node',
        output='screen'
    )
    
    ld.add_action(pure_pursuit_node)
    
    return ld
    
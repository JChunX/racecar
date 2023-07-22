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
        'frenet.yaml'
    )
    
    rrt_node = Node(
        package='lab6_pkg',
        executable='frenet_node',
        name='frenet_node',
        parameters=[config],
        output='screen'
    )
    
    drive_node = Node(
        package='lab6_pkg',
        executable='drive_node',
        name='drive_node',
        parameters=[config],
        output='screen'
    )
    
    ld.add_action(rrt_node)
    ld.add_action(drive_node)
    
    return ld
    
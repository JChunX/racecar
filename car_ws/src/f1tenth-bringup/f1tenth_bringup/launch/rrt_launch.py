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
        'rrt.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    
    rrt_node = Node(
        package='lab6_pkg',
        executable='rrt_node',
        name='rrt_node',
        parameters=[config],
        output='screen'
    )
    
    ld.add_action(rrt_node)
    
    return ld
    
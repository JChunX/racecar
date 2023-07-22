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
        'mpc.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    
    mpc_node = Node(
        package='mpc',
        executable='mpc_node',
        parameters=[config],
        name='mpc_node',
        output='screen'
    )
    
    ld.add_action(mpc_node)
    
    return ld
    
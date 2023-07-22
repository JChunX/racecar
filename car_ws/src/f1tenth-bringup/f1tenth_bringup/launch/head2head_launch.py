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
    
    ego_plan_node = Node(
        package='lab6_pkg',
        executable='lane_change_node',
        name='lane_change_node',
        parameters=[config, {'is_ego': True}],
        output='screen'
    )
    
    opp_plan_node = Node(
        package='lab6_pkg',
        executable='frenet_node',
        name='frenet_node',
        parameters=[config, {'is_ego': False}],
        output='screen'
    )
    
    ego_drive_node = Node(
        package='lab6_pkg',
        executable='drive_node',
        name='drive_node',
        parameters=[config, {'is_ego': True}],
        output='screen'
    )
    
    opp_drive_node = Node(
        package='lab6_pkg',
        executable='drive_node',
        name='drive_node',
        parameters=[config, {'is_ego': False}],
        output='screen'
    )
    
    ld.add_action(ego_plan_node)
    #ld.add_action(opp_plan_node)
    ld.add_action(ego_drive_node)
    ld.add_action(opp_drive_node)
    
    return ld
    
    
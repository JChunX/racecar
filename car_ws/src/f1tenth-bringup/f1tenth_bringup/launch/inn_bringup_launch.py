import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'real.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))
    
    stack_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth_stack'),
                'launch',
                'bringup_launch.py'
            ])
        )
    )
    
    parameter_server = Node(
        package='f1tenth_bringup',
        executable='global_parameter_server',
        name='global_parameter_server',
        parameters=[config]
    )
    
    logger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('f1tenth_bringup'),
                'launch',
                'logging_launch.py'
            ])
        )
    )
    
    drive_republish_node = Node(
        package='drive_republisher',
        executable='republisher',
        name='drive_republisher'
    )
    
    do_localize = config_dict['localize']['ros__parameters']['do_localize']
    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('local_inn'),
                'launch',
                'localize_launch.py'
            ])
        )
    )
    
    waypoint_publisher = Node(
        package='pure_pursuit',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        parameters=[config],
    )
    
    if do_localize:
        ld.add_action(localizer)
    ld.add_action(stack_bringup)
    ld.add_action(logger)
    ld.add_action(drive_republish_node)
    ld.add_action(parameter_server)
    ld.add_action(waypoint_publisher)
    
    return ld
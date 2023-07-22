import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'map_env.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    
    slam_param_path = config_dict['slam']['slam_param_path']
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
            ])
        ),
        launch_arguments={'params_file': slam_param_path}.items(),
        #remappings=[('/scan', '/scan_filtered')]
    )
    # convert this shell ros2 command to Node() action: 
    # ros2 run nav2_map_server map_saver_cli -f levine_slam
    map_path = os.path.join(get_package_share_directory('f1tenth_bringup'), 'maps', 'levine_slam')
    
    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        parameters=[config],
        arguments=['-f', map_path],
        output='screen'
    )
    
    ld.add_action(slam)
    
    return ld
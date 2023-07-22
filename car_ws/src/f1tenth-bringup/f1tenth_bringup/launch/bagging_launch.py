import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'bagging.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))

    now = datetime.now()
    date_time = now.strftime("%Y-%m-%d-%H-%M-%S")
    home_dir = os.path.expanduser('~')
    bag_path = config_dict['rosbag']['bag_path']
    save_path = os.path.join(home_dir, bag_path, date_time)
    
    rosbag = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', save_path])
    
    ld.add_action(rosbag)
    
    return ld
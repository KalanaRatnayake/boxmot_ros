import os

import ament_index_python.packages
import launch_ros.actions
from launch import LaunchDescription

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('boxmot_ros')
    
    params_file = os.path.join(share_dir, 'config', 'boxmot_ros_params.yaml')

    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['boxmot_ros_node']['ros__parameters']

    boxmot_ros_node = launch_ros.actions.Node(package='boxmot_ros',
                                              executable='boxmot_ros',
                                              output='both',
                                              parameters=[params]
                                              )

    ld = LaunchDescription()

    ld.add_action(boxmot_ros_node)

    return ld
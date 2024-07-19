import os

import ament_index_python.packages
import launch_ros.actions
from launch import LaunchDescription

import yaml


def generate_launch_description():
    yolo_ros_node = launch_ros.actions.Node(package='yolo_ros',
                                              executable='yolo_ros',
                                              output='both'
                                              )

    boxmot_ros_node = launch_ros.actions.Node(package='boxmot_ros',
                                              executable='boxmot_ros',
                                              output='both'
                                              )

    ld = LaunchDescription()

    ld.add_action(yolo_ros_node)
    ld.add_action(boxmot_ros_node)

    return ld
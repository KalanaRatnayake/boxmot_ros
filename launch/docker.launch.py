import os

import ament_index_python.packages
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import yaml


def generate_launch_description():
    yolo_yolo_model_param                = DeclareLaunchArgument('yolo_model',                      default_value="yolov9s.pt")
    yolo_input_rgb_topic_param           = DeclareLaunchArgument('input_rgb_topic',                 default_value="/camera/color/image_raw")
    yolo_input_depth_topic_param         = DeclareLaunchArgument('input_depth_topic',               default_value="/camera/depth/points")
    yolo_subscribe_depth_param           = DeclareLaunchArgument('subscribe_depth',                 default_value=True)
    yolo_publish_annotated_image_param   = DeclareLaunchArgument('yolo_publish_annotated_image',    default_value=True)
    yolo_annotated_topic_param           = DeclareLaunchArgument('yolo_annotated_topic',            default_value="/yolo_ros/annotated_image")
    yolo_detailed_topic_param            = DeclareLaunchArgument('yolo_detailed_topic',             default_value="/yolo_ros/detection_result")
    yolo_threshold_param                 = DeclareLaunchArgument('yolo_threshold',                  default_value=0.25)
    yolo_device_param                    = DeclareLaunchArgument('yolo_device',                     default_value="0")

    boxmot_yolo_model_param              = DeclareLaunchArgument('tracking_model',                  default_value="deepocsort")
    boxmot_input_rgb_topic_param         = DeclareLaunchArgument('reid_model',                      default_value="osnet_x0_25_msmt17.pt")
    boxmot_input_depth_topic_param       = DeclareLaunchArgument('input_topic',                     default_value="/yolo_ros/detection_result")
    boxmot_publish_annotated_image_param = DeclareLaunchArgument('boxmot_publish_annotated_image',  default_value=True)
    boxmot_annotated_topic_param         = DeclareLaunchArgument('boxmot_annotated_topic',          default_value="/boxmot_ros/annotated_image")
    boxmot_detailed_topic_param          = DeclareLaunchArgument('boxmot_detailed_topic',           default_value="/boxmot_ros/tracking_result")
    boxmot_threshold_param               = DeclareLaunchArgument('boxmot_threshold',                default_value=0.25)
    boxmot_device_param                  = DeclareLaunchArgument('boxmot_device',                   default_value="0")

    yolo_ros_node = launch_ros.actions.Node(package='yolo_ros',
                                              executable='yolo_ros',
                                              output='both',
                                              parameters=[{
                                                  'yolo_model': LaunchConfiguration('yolo_model'),
                                                  'input_rgb_topic': LaunchConfiguration('input_rgb_topic'),
                                                  'input_depth_topic': LaunchConfiguration('input_depth_topic'),
                                                  'subscribe_depth': LaunchConfiguration('subscribe_depth'),
                                                  'publish_annotated_image': LaunchConfiguration('yolo_publish_annotated_image'),
                                                  'annotated_topic': LaunchConfiguration('yolo_annotated_topic'),
                                                  'detailed_topic': LaunchConfiguration('yolo_detailed_topic'),
                                                  'threshold': LaunchConfiguration('yolo_threshold'),
                                                  'device': LaunchConfiguration('yolo_device'),
                                              }]
                                              )

    boxmot_ros_node = launch_ros.actions.Node(package='boxmot_ros',
                                              executable='boxmot_ros',
                                              output='both',
                                              parameters=[{
                                                  'tracking_model': LaunchConfiguration('tracking_model'),
                                                  'reid_model': LaunchConfiguration('reid_model'),
                                                  'input_topic': LaunchConfiguration('input_topic'),
                                                  'publish_annotated_image': LaunchConfiguration('boxmot_publish_annotated_image'),
                                                  'annotated_topic': LaunchConfiguration('boxmot_annotated_topic'),
                                                  'detailed_topic': LaunchConfiguration('boxmot_detailed_topic'),
                                                  'threshold': LaunchConfiguration('boxmot_threshold'),
                                                  'device': LaunchConfiguration('boxmot_device')
                                              }]
                                              )

    ld = LaunchDescription()

    ld.add_action(yolo_yolo_model_param)
    ld.add_action(yolo_input_rgb_topic_param)
    ld.add_action(yolo_input_depth_topic_param)
    ld.add_action(yolo_subscribe_depth_param)
    ld.add_action(yolo_publish_annotated_image_param)
    ld.add_action(yolo_annotated_topic_param)
    ld.add_action(yolo_detailed_topic_param)
    ld.add_action(yolo_threshold_param)
    ld.add_action(yolo_device_param)

    ld.add_action(boxmot_yolo_model_param)
    ld.add_action(boxmot_input_rgb_topic_param)
    ld.add_action(boxmot_input_depth_topic_param)
    ld.add_action(boxmot_publish_annotated_image_param)
    ld.add_action(boxmot_annotated_topic_param)
    ld.add_action(boxmot_detailed_topic_param)
    ld.add_action(boxmot_threshold_param)
    ld.add_action(boxmot_device_param)

    ld.add_action(yolo_ros_node)
    ld.add_action(boxmot_ros_node)

    return ld
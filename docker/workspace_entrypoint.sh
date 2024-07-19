#!/bin/bash
set -e

if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
	echo "sourcing /opt/ros/$ROS_DISTRO/setup.bash"
	source /opt/ros/$ROS_DISTRO/setup.bash
else
	echo "notfound /opt/ros/$ROS_DISTRO/setup.bash"

    echo "sourcing /opt/ros/$ROS_DISTRO/install/setup.bash"
    source /opt/ros/$ROS_DISTRO/install/setup.bash
fi	

echo "sourcing $WORKSPACE_ROOT/install/setup.bash"
source "$WORKSPACE_ROOT/install/setup.bash"

ros2 param set /boxmot_ros tracking_model $TRACKING_MODEL
ros2 param set /boxmot_ros reid_model $REID_MODEL
ros2 param set /boxmot_ros input_topic $INPUT_TOPIC
ros2 param set /boxmot_ros publish_annotated_image $PUBLISH_ANNOTATED_IMAGE
ros2 param set /boxmot_ros annotated_topic $BOXMOT_ANNOTATED_TOPIC
ros2 param set /boxmot_ros detailed_topic $BOXMOT_DETAILED_TOPIC
ros2 param set /boxmot_ros threshold $BOXMOT_THRESHOLD
ros2 param set /boxmot_ros device $BOXMOT_DEVICE

ros2 param set /yolo_ros yolo_model $YOLO_MODEL
ros2 param set /yolo_ros input_rgb_topic $INPUT_RGB_TOPIC
ros2 param set /yolo_ros input_depth_topic $INPUT_DEPTH_TOPIC
ros2 param set /yolo_ros subscribe_depth $SUBSCRIBE_DEPTH
ros2 param set /yolo_ros publish_annotated_image $PUBLISH_ANNOTATED_IMAGE
ros2 param set /yolo_ros annotated_topic $YOLO_ANNOTATED_TOPIC
ros2 param set /yolo_ros detailed_topic $YOLO_DETAILED_TOPIC
ros2 param set /yolo_ros threshold $YOLO_THRESHOLD
ros2 param set /yolo_ros device $YOLO_DEVICE

ros2 param list

exec "$@"

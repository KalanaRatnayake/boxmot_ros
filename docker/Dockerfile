#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:humble-ros-base-jammy as base

## Parameters
ENV WORKSPACE_ROOT=/boxmot

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

WORKDIR /

RUN apt-get update -y

RUN apt-get install -y --no-install-recommends  python3-pip \
                                                libboost-python-dev \
                                                ros-dev-tools \
                                                ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
                                                ros-$ROS_DISTRO-vision-opencv

RUN pip3 install --upgrade --no-cache-dir torch==1.13.0+cu117 \
                                          torchvision==0.14.0+cu117 \
                                          torchaudio==0.13.0 \
                                          --extra-index-url https://download.pytorch.org/whl/cu117

RUN apt-get clean

#############################################################################################################################
#####
#####   Clone and build boxmot packages
#####
#############################################################################################################################

WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/KalanaRatnayake/boxmot_ros.git
RUN git clone https://github.com/KalanaRatnayake/yolo_ros.git
RUN git clone https://github.com/KalanaRatnayake/detection_msgs.git

RUN pip3 install --no-cache-dir -r ${WORKSPACE_ROOT}/src/boxmot_ros/requirements.txt
RUN pip3 install --no-cache-dir -r ${WORKSPACE_ROOT}/src/yolo_ros/requirements.txt

RUN rosdep install --from-paths ${WORKSPACE_ROOT}/src -y --ignore-src

WORKDIR ${WORKSPACE_ROOT}

RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /

#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevent to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*

RUN apt-get clean

#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------


FROM ros:humble-ros-base-jammy as final

## Parameters

ENV WORKSPACE_ROOT=/boxmot

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /

COPY --from=base / /

COPY docker/workspace_entrypoint.sh /workspace_entrypoint.sh

RUN chmod +x /workspace_entrypoint.sh

ENTRYPOINT [ "/workspace_entrypoint.sh" ]

WORKDIR ${WORKSPACE_ROOT}

CMD ros2 launch boxmot_ros docker.launch.py yolo_model:=$YOLO_MODEL \
                                            input_rgb_topic:=$INPUT_RGB_TOPIC \
                                            input_depth_topic:=$INPUT_DEPTH_TOPIC \
                                            subscribe_depth:=$SUBSCRIBE_DEPTH \
                                            yolo_publish_annotated_image:=$YOLO_PUBLISH_ANNOTATED_IMAGE \
                                            yolo_annotated_topic:=$YOLO_ANNOTATED_TOPIC \
                                            yolo_detailed_topic:=$YOLO_DETAILED_TOPIC \
                                            yolo_threshold:=$YOLO_THRESHOLD \
                                            yolo_device:=$YOLO_DEVICE \
                                            tracking_model:=$TRACKING_MODEL \
                                            reid_model:=$REID_MODEL \
                                            input_topic:=$INPUT_TOPIC \
                                            boxmot_publish_annotated_image:=$BOXMOT_PUBLISH_ANNOTATED_IMAGE \
                                            boxmot_annotated_topic:=$BOXMOT_ANNOTATED_TOPIC \
                                            boxmot_detailed_topic:=$BOXMOT_DETAILED_TOPIC \
                                            boxmot_threshold:=$BOXMOT_THRESHOLD \
                                            boxmot_device:=$BOXMOT_DEVICE

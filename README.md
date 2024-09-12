# boxmot_ros

## Docker Usage by adding to compose.yml file

To use GPU with docker while on AMD64 systems, install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) with given instructions.

### Supported platforms

Replace `image` and 'device' parameter in the compose.yml with following values for respective systems.

| System      | ROS Version | Value for `image`                                 | Value for `device`  | Size    | file  |
| :---        | :---        | :---                                              |  :---               | :---:   | :---: |
| AMD64       | Humble      | ghcr.io/kalanaratnayake/boxmot-ros:humble         | `cpu`, `0`, `0,1,2` | 5.7 GB  | docker/compose.amd64.yaml |

## Docker Usage with this repository

Clone this reposiotory

```bash
mkdir -p boxmot_ws/src && cd boxmot_ws/src
git clone https://github.com/KalanaRatnayake/boxmot_ros.git && cd ..
```

<details> 
<summary> <h3> on AMD64 </h3> </summary>
  
Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd src/boxmot_ros/docker
docker compose -f compose.amd64.yaml pull
docker compose -f compose.amd64.yaml up
```

Reset the system and remove volume
```bash
docker compose -f compose.amd64.yaml down
docker volume rm docker_boxmot
```

</details>


<br>

## Native Usage

Clone this repository with and install dependencies.

```bash
git clone https://github.com/KalanaRatnayake/boxmot_ros.git
git clone https://github.com/KalanaRatnayake/detection_msgs.git
pip3 install -r boxmot_ros/requirements.txt

cd ..
rosdep install --from-paths src -y --ignore-src
```

### Build the package

If required, edit the parameters at `config/boxmot_ros_params.yaml' and then at the workspace root run,
```bash
colcon build
```
### Start the system

To use the boxmot tracking, run the launch file in terminal, run,

```bash
source ./install/setup.bash
ros2 launch boxmot_ros boxmot.launch.py
```

<br>
<br>

## Parameter decription

| ROS Parameter           | Docker ENV parameter    | Default Value                 | Description |
| :---                    | :---                    | :---:                         | :---        |
| yolo_model              | YOLO_MODEL              | `yolov9t.pt`                  | Model to be used. see [1] for default models and [2] for custom models |
| tracking_model          | TRACKING_MODEL          | `deepocsort`                  | Model to be used for tracking. see [3] for default models |
| reid_model              | REID_MODEL              | `osnet_x0_25_msmt17.pt`       | Model to be used for reidentification. see [3] for default models |
| subscribe_depth         | SUBSCRIBE_DEPTH         | `False`                       | Whether to subscribe to depth image or not. This will also enable the depth_topic variable which publishes synchronized depth image. Use if having a depth camera. A ApproximateTimeSynchronizer is used to sync RGB and Depth images |
| input_rgb_topic         | INPUT_RGB_TOPIC         | `/camera/color/image_raw`     | Topic to subscribe for RGB image. Accepts `sensor_msgs/Image` |
| input_depth_topic       | INPUT_DEPTH_TOPIC       | `/camera/depth/points`        | Topic to subscribe for Depth image. Accepts `sensor_msgs/PointCloud2` |
| publish_annotated_image | PUBLISH_ANNOTATED_IMAGE | `False`                       | Whether to publish tracking annotated image, increases callback execution time when set to `True` |
| rgb_topic               | RGB_TOPIC        | `/boxmot_ros/rgb_image`       | Topic for publishing synchronized rgb images. uses `sensor_msgs/Image` |
| depth_topic             | DEPTH_TOPIC      | `/boxmot_ros/depth_image`     | Topic for publishing synchronized depth images. uses `detection_msgs/PointCloud2` |
| annotated_topic         | ANNOTATED_TOPIC  | `/boxmot_ros/annotated_image` | Topic for publishing annotated images uses `sensor_msgs/Image` |
| detailed_topic          | DETAILED_TOPIC   | `/boxmot_ros/tracking_result` | Topic for publishing detailed results uses `detection_msgs/Detections` |
| threshold               | THRESHOLD        | `0.25`                        | Confidence threshold for predictions |
| device                  | DEVICE           | `'0'`                         | `cpu` for CPU, `0` for gpu, `0,1,2,3` if there are multiple GPUs |


[1] If the model is available at [ultralytics models](https://docs.ultralytics.com/models/), It will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup.

[2] Uncomment the commented out `YOLO_MODEL` parameter line and give the custom model weight file's name as `YOLO_MODEL` parameter. Uncomment the docker bind entry that to direct to the `weights` folder and comment the docker volume entry for yolo. Copy the custom weights to the `weights` folder.

[3] If the reid model is available at [MODEL_ZOO](https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO), and tracking_model is supported [`deepocsort`, `strongsort`, `ocsort`, `bytetrack`, `botsort`, `hybridsort`, `imprassoc`]. They will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup. Use the snipped in [Default models with Docker Compose](https://github.com/KalanaRatnayake/boxmot_ros#default-models-with-docker-compose)

## Latency description

Here is a summary of whether supported models work with boxmot_ros node (in docker) on various platforms and the time it takes to execute a single interation of `BoxmotROS.image_callback` function. Values are measured as an average of 100 executions of the function and Input is a 640x480 RGB image at 30 fps.

Laptop -> Ryzen 9 16 core with RTX3070 mobile GPU with Ubuntu 22.04

| Model | Laptop (amd64) |
| :---  |  ---: |
| `deepocsort` | 27 ms |
| `strongsort` | 20 ms |
| `ocsort`     | 17 ms |
| `bytetrack`  | 17 ms |
| `botsort`    | 14 ms |

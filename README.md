# boxmot_ros

## Docker Usage by adding to compose.yml file

To use GPU with docker while on AMD64 systems, install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) with given instructions.

### Supported platforms

Replace `image` and 'device' parameter in the compose.yml with following values for respective systems.

| System      | ROS Version | Value for `image`                                 | Value for `device`  | Size    | file  |
| :---        | :---        | :---                                              |  :---               | :---:   | :---: |
| AMD64       | Humble      | ghcr.io/kalanaratnayake/boxmot-ros:humble         | `cpu`, `0`, `0,1,2` | 5.7 GB  | docker/compose.amd64.yaml |
| Jetson Nano | Humble      | ghcr.io/kalanaratnayake/boxmot-ros:humble-j-nano  | `cpu`, `0`          | 3.36GB  | docker/compose.jnano.yaml |

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

<details> 
<summary> <h3> on JetsonNano </h3> </summary>
  
Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
cd src/boxmot_ros/docker
docker compose -f compose.jnano.yaml pull
docker compose -f compose.jnano.yaml up
```

Reset the system and remove volume
```bash
docker compose -f compose.jnano.yaml down
docker volume rm docker_boxmot
```
</details>

<br>

## Native Usage

Clone this repository with and install dependencies.

```bash
git clone https://github.com/KalanaRatnayake/boxmot_ros.git
git clone https://github.com/KalanaRatnayake/yolo_ros.git
git clone https://github.com/KalanaRatnayake/detection_msgs.git
pip3 install -r boxmot_ros/requirements.txt
pip3 install -r yolo_ros/requirements.txt

cd ..
rosdep install --from-paths src -y --ignore-src
```

### Build the package

If required, edit the parameters at `config/boxmot_ros_params.yaml' and then at the workspace root run,
```bash
colcon build
```
### Start the system

To use the yolo detection, run launch file in a terminal,

```bash
source ./install/setup.bash
ros2 launch yolo_ros yolo.launch.py
```

To use the boxmot tracking, run the launch file in a seperate terminal, run,

```bash
source ./install/setup.bash
ros2 launch boxmot_ros boxmot.launch.py
```

<br>
<br>

## Parameter decription

For parameters missing in thefollowing table (`yolo_ros` parameters) check the respective [yolo_ros parameters](https://github.com/KalanaRatnayake/yolo_ros#parameter-decription).

| ROS Parameter           | Docker ENV parameter    | Default Value                 | Description |
| :---                    | :---                    | :---:                         | :---        |
| tracking_model          | TRACKING_MODEL          | `deepocsort`                  | Model to be used for tracking. see [1] for default models |
| reid_model              | REID_MODEL              | `osnet_x0_25_msmt17.pt`       | Model to be used for reidentification. see [1] for default models |
| input_topic             | INPUT_TOPIC             | `/yolo_ros/detection_result`  | Topic to subscribe for RGB image. Accepts `sensor_msgs/Image` |
| publish_tracking_image  | PUBLISH_ANNOTATED_IMAGE | `False`                       | Whether to publish tracking annotated image, increases callback execution time when set to `True` |
| annotated_topic         | BOXMOT_ANNOTATED_TOPIC  | `/boxmot_ros/annotated_image` | Topic for publishing annotated images uses `sensor_msgs/Image` |
| detailed_topic          | BOXMOT_DETAILED_TOPIC   | `/boxmot_ros/tracking_result` | Topic for publishing detailed results uses `detection_msgs/Detections` |
| threshold               | BOXMOT_THRESHOLD        | `0.25`                        | Confidence threshold for predictions |
| device                  | BOXMOT_DEVICE           | `'0'`                         | `cpu` for CPU, `0` for gpu, `0,1,2,3` if there are multiple GPUs |


[1] If the reid model is available at [MODEL_ZOO](https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO), and tracking_model is supported [deepocsort, strongsort, ocsort, bytetrack, botsort]. They will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup. Use the snipped in [Default models with Docker Compose](https://github.com/KalanaRatnayake/boxmot_ros#default-models-with-docker-compose)

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

# boxmot_ros

## Docker Usage by adding to compose.yml file

To use GPU with docker while on AMD64 systems, install [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) with given instructions.

### Supported platforms

Replace `image` and 'device' parameter in the compose.yml with following values for respective systems.

| System              | ROS Version | Value for `image`                                 | Value for `device`  | Size    |
| :---                | :---        | :---                                              |  :---               | :---:   |
| AMD64               | Humble      | ghcr.io/kalanaratnayake/boxmot-ros:humble         | `cpu`, `0`, `0,1,2` | 5.64 GB |

### Default models with Docker Compose

Add the following snippet under `services` to any compose.yaml file to add this container and use an existing model.

```bash
services:
  boxmot:
    image: ghcr.io/kalanaratnayake/boxmot-ros:humble
    environment:
      - TRACKING_MODEL=deepocsort
      - REID_MODEL=osnet_x0_25_msmt17.pt
      - INPUT_TOPIC=/yolo_ros/detection_result
      - PUBLISH_ANNOTATED_IMAGE=True
      - OUTPUT_ANNOTATED_TOPIC=/boxmot_ros/annotated_image
      - OUTPUT_DETAILED_TOPIC=/boxmot_ros/tracking_result
      - CONFIDENCE_THRESHOLD=0.25
      - DEVICE='0'
    restart: unless-stopped
    privileged: true
    network_mode: host
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]   
    volumes:
      - boxmot:/boxmot

volumes:
  boxmot:
```

## Docker Usage with this repository

Clone this reposiotory

```bash
mkdir -p boxmot_ws/src && cd boxmot_ws/src
git clone https://github.com/KalanaRatnayake/boxmot_ros.git && cd ..
```

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

<br>

## Native Usage

Clone this repository with and install dependencies.

```bash
git clone https://github.com/KalanaRatnayake/boxmot_ros.git
git clone https://github.com/KalanaRatnayake/boxmot_ros_msgs.git
cd boxmot_ros
pip3 install -r requirements.txt
```

### Build the package

If required, edit the parameters at `config/boxmot_ros_params.yaml' and then at the workspace root run,
```bash
colcon build
```
### Start the system

To use the launch file, run,

```bash
source ./install/setup.bash
ros2 launch boxmot_ros boxmot.launch.py
```

<br>
<br>

## Parameter decription

| ROS Parameter           | Docker ENV parameter    | Default Value                | Description |
| :---                    | :---                    | :---:                        | :---        |
| tracking_model          | TRACKING_MODEL          | `deepocsort`                 | Model to be used for tracking. see [1] for default models and [2] for custom models |
| reid_model              | REID_MODEL              | `osnet_x0_25_msmt17.pt`      | Model to be used for reidentification. see [1] for default models and [2] for custom models |
| input_topic             | INPUT_TOPIC             | `/yolo_ros/detection_result` | Topic to subscribe for RGB image. Accepts `sensor_msgs/Image` |
| publish_annotated_image | PUBLISH_ANNOTATED_IMAGE | `False`                      | Whether to publish annotated image, increases callback execution time when set to `True` |
| output_annotated_topic  | OUTPUT_ANNOTATED_TOPIC  | `/boxmot_ros/annotated_image` | Topic for publishing annotated images uses `sensor_msgs/Image` |
| output_detailed_topic   | OUTPUT_DETAILED_TOPIC   | `/boxmot_ros/tracking_result` | Topic for publishing detailed results uses `boxmot_ros_msgs/YoloResult` |
| confidence_threshold    | CONFIDENCE_THRESHOLD    | `0.25`                      | Confidence threshold for predictions |
| device                  | DEVICE                  | `'0'`                       | `cpu` for CPU, `0` for gpu, `0,1,2,3` if there are multiple GPUs |


[1] If the reid model is available at [MODEL_ZOO](https://kaiyangzhou.github.io/deep-person-reid/MODEL_ZOO), and tracking_model is supported [deepocsort, strongsort, ocsort, bytetrack, botsort]. They will be downloaded from the cloud at the startup. We are using docker volumes to maintain downloaded weights so that weights are not downloaded at each startup. Use the snipped in [Default models with Docker Compose](https://github.com/KalanaRatnayake/boxmot_ros#default-models-with-docker-compose)

[2] Give the tracking model weight file's name as `TRACKING_MODEL` parameter and reidentification model weight file's name as `REID_MODEL` parameter. Update the docker volume source tag to direct to the folder and use docker bind-mounts instead of docker volumes where the weight files exist in the host machine. As an example if the weight files are in `/home/kalana/Downloads/weight/` folder, then use the snipped in [Custom models with Docker Compose](https://github.com/KalanaRatnayake/boxmot_ros#custom-models-with-docker-compose)

## Latency description

Here is a summary of whether supported models work with boxmot_ros node (in docker) on various platforms and the time it takes to execute a single interation of `YoloROS.image_callback` function. Values are measured as an average of 100 executions of the function and Input is a 640x480 RGB image at 30 fps.

Laptop -> Ryzen 9 16 core with RTX3070 mobile GPU with Ubuntu 22.04

| Model | Laptop (amd64) |
| :---  |  ---: |
| `deepocsort` | 27 ms |
| `strongsort` | 20 ms |
| `ocsort`     | 17 ms |
| `bytetrack`  | 17 ms |
| `botsort`    | 14 ms |

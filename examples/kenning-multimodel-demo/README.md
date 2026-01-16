# Execution of multiple models in parallel using ROS 2 and Kenning

This demo runs computer vision models in separate ROS 2 nodes instantiated using Kenning:

* Instance segmentation of objects using [Yolact](https://github.com/dbolya/yolact) - using [`kenning-instance-segmentation.yaml`](./kenning-instance-segmentation.yaml) scenario.
* Human pose estimation for found people in view using [MMPose RTMPose](https://github.com/open-mmlab/mmpose) - using [`kenning-pose-from-detection.yaml`](./kenning-pose-from-detection.yaml) scenario.
* Depth estimation using [DINOv2](https://github.com/facebookresearch/dinov2) - for depth estimation, [`kenning-depth-estimation.yaml`](./kenning-depth-estimation.yaml) scenario.

In addition to above Kenning-based ROS 2 nodes, the application also runs:

* [Camera node](https://github.com/antmicro/ros2-camera-node) - reads data from a camera and exposes its settings
* [GUI node](https://github.com/antmicro/ros2-gui-node) - renders a window with camera view and outputs from Kenning nodes.

> **NOTE**
>
> For instruction on how to run demo on **Nvidia Jetson platform**
> look into [jetson/README.md](./jetson/README.md) folder.

# Running on x86 based systems

## Quickstart

You can pull demo image using:

``` bash
docker pull ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo
```

Then create directory for demo

```bash
mkdir kenning-ros2-demo && cd kenning-ros2-demo
```

and prepare repositories:

```bash
repo init -u git@github.com:antmicro/ros2-gui-node.git -m examples/kenning-multimodel-demo/manifest.xml

repo sync -j`nproc`
```

Then you can execute demo by simply running:

``` bash
src/gui_node/examples/kenning-multimodel-demo/run-demo.sh
```

## Setting up an environment

> **NOTE**
>
> This demo requires:
> 
> * A camera
> * A CUDA-enabled NVIDIA GPU for inference acceleration
> * A [git](https://git-scm.com/) version control system
> * [repo tool](https://gerrit.googlesource.com/git-repo/+/refs/heads/main/README.md) to clone all necessary repositories
> * [Docker](https://www.docker.com/) to use a prepared ROS 2 environment
> * [nvidia-container-toolkit](https://github.com/nvidia/nvidia-container-toolkit) to provide access to the GPU in the Docker container

Ready to use ROS 2 environment can be found in `ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo` Docker image (defined in [project's Dockerfile](../../environments/Dockerfile)).

Built image can be downloaded like so:

```
docker pull ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo
```

The image can be built from scratch by running a following command in the [environments](../../environments/) directory:

```
sudo ./build-docker.sh
```

## Downloading the demo

First off, create a workspace directory, where downloaded repositories will be stored:

```bash
mkdir kenning-ros2-demo && cd kenning-ros2-demo
```

Then, download all dependencies using the `repo` tool:

```bash
repo init -u git@github.com:antmicro/ros2-gui-node.git -m examples/kenning-multimodel-demo/manifest.xml

repo sync -j`nproc`
```

> **NOTE** 
>
> Before executing repo command you may need to set up git credential by typing into terminal:
>
> ``` bash
> git config --global user.email "you@example.com"
> git config --global user.name "Your Name"
> ```
> It can be dummy credential, it doesn't have to be real user credential.

## Starting the Docker environment

If you use the Docker container, allow non-network local connections to access X11 so that GUI applications can be started from the Docker container:

``` bash
xhost +local:
```

After this, run a Docker container under the `kenning-ros2-demo` directory with:

``` bash
sudo ./src/gui_node/environments/run-docker.sh
```

> **NOTE** 
>
> In case you have built the image manually, e.g. with name `kenning-ros2-demo`, run `DOCKER_IMAGE=kenning-ros2-demo ./run-docker.sh`.
> Also, if you want to change the camera path (default `/dev/video0`), set the `CAMERA_PATH` variable with your desired path before running the script.

Scripts checks for the presence of NVIDIA drivers - if NVIDIA GPU is not present, the container will run in CPU-only mode.
If you want to explicitly run the container without GPU acceleration, run:

``` bash
sudo ../run-docker.sh cpu
```

This script starts the container with:

* `--device=/dev/video0:/dev/video0` - adds a camera device to the container's context
* `-v $(pwd):/data` - mounts current (`kenning-ros2-demo`) directory in the `/data` directory in the container's context
* `-v /tmp/.X11-unix/:/tmp/.X11-unix/` - passes the X11 socket directory to the container's context (to allow running GUI application)
* `-e DISPLAY=$DISPLAY`, `-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR` - adds X11-related environment variables
* `--gpus='all,"capabilities=compute,utility,graphics,display"'` - adds GPUs to the container's context for computing and displaying purposes

Then, in the Docker container, you need to install graphics libraries for NVIDIA that match your host's drivers.
To check NVIDIA drivers version, run:

```
nvidia-smi
```

And check the `Driver version`.

For example, for 530.41.03, install the following libraries in the container:

```
apt-get update && apt-get install libnvidia-gl-530
```

Then, go to the workspace directory in the container:

```
cd /data
```
## Install Kenning

Install **Kenning** with necessary dependencies:

You may update pip before processing, to avoid installation errors:

``` bash
pip install --upgrade pip
```

``` bash
pip install "./kenning[object_detection,pose_estimation,onnxruntime,onnxruntime_gpu]"
```

## Building GUI node and Camera node

First of all, load the `setup.sh` script for ROS 2 tools in Docker container, e.g.:

```bash
source /opt/ros/setup.sh
```

Then, build the GUI node and the Camera node with:

```bash
colcon build --base-paths src --cmake-args -DBUILD_KENNING_MULTIMODEL_DEMO=y
```

## Running the demo with a camera

Then, to run the demo, load the ROS 2 environment including the newly built packages:

```bash
source install/setup.sh
```

Next, launch Kenning, Camera node, and GUI node using the launch file [`kenning-multimodel-demo.py use_gui:=True`](./kenning-multimodel-demo.py):

> **NOTE** 
>
> If you want to run demo using local model's backups set `MODEL_PATH`
> environment variable with directory containing models ONNX file with
> thier respected configs

```
ros2 launch gui_node kenning-multimodel-demo.py use_gui:=True
```

In case you want to change the path to the camera, use the `camera_path:=<new-path>` argument, e.g.:

```
ros2 launch gui_node kenning-multimodel-demo.py camera_path:=/dev/video1
```

Lastly, a GUI should appear, with the:

* Direct view from camera node
* Instance segmentation view based on predictions from Kenning (started using `kenning flow` with [`kenning-instance-segmentation.yaml`](./kenning-instance-segmentation.yaml)).
* Pose detection view based on predictions from Kenning (started using `kenning flow` with [`kenning-pose-from-detection.yaml`](./kenning-pose-from-detection.yaml)).
* Depth estimation view based on predictions from Kenning (started using `kenning flow` with [`kenning-depth-estimation.yaml`](./kenning-depth-estimation.yaml))
* A widget visualizing a list of detected objects, with a possibility to filter out not interesting classes.
* A widget showing estimated poses for all persons detected on the image.
* A widget for showing depth estimation results.
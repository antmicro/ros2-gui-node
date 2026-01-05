# Execution of multiple models in parallel using ROS 2 and Kenning

This demo runs computer vision models in separate ROS 2 nodes instantiated using Kenning:

* Instance segmentation of objects using [Yolact](https://github.com/dbolya/yolact) - using [`kenning-instance-segmentation.yaml`](./kenning-instance-segmentation.yaml) scenario.
* Human pose estimation for found people in view using [MMPose RTMPose](https://github.com/open-mmlab/mmpose) - using [`kenning-pose-from-detection.yaml`](./kenning-pose-from-detection.yaml) scenario.
* Depth estimation using [DINOv2](https://github.com/facebookresearch/dinov2) - for depth estimation, [`kenning-depth-estimation.yaml`](./kenning-depth-estimation.yaml) scenario.

In addition to above Kenning-based ROS 2 nodes, the application also runs:

* [Camera node](https://github.com/antmicro/ros2-camera-node) - reads data from a camera and exposes its settings
* [GUI node](https://github.com/antmicro/ros2-gui-node) - renders a window with camera view and outputs from Kenning nodes.

# Running on Nvidia Jetson 

## Setting up an environment

> **NOTE**
>
> This demo requires:
> 
> * A camera
> * A CUDA-enabled NVIDIA GPU for inference acceleration
> * A [git](https://git-scm.com/) version control system
> * [repo tool](https://gerrit.googlesource.com/git-repo/+/refs/heads/main/README.md) to clone all necessary repositories
> * NVIDIA Container Toolkit
> * NVIDIA Jetson JetPack
> * Docker
>
> For instructions on how to install them look into: 
> * https://docs.nvidia.com/jetson/jetpack/install-setup/index.html
> * https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/setup_docker.html

First make sure you have installed **jetson-containers** https://github.com/dusty-nv/jetson-containers.
To install it type into terminal:
``` bash
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```

Then go to **ros2-gui-node** repository and go to the folder [environments/jetson](../../environments/jetson) and
execute:
``` bash
sudo ./build-docker.sh
```
> **NOTE**  
> 
> Due to bug: https://github.com/dusty-nv/jetson-containers/issues/1577, in jetson-containers you may encounter a build error halfway through, something like:
>```
>  Error: Command 'docker run ...
>
>  ...Failed building: cuda
>```
>but it is expected behaviour.

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

It can be dummy credential, it doesn't have to be real user credential.

## Starting the Docker environment

If you use the Docker container, allow non-network local connections to access X11 so that GUI applications can be started from the Docker container:

``` bash
xhost +local:
```

After this, run a Docker container under the `kenning-ros2-demo` directory with:

``` bash
sudo ./src/gui_node/environments/jetson/run-docker.sh
```

> **NOTE** 
>
> If you want to change the camera path (default `/dev/video0`), set the `CAMERA_PATH` variable with your desired path before running the script.

This script starts the container with:

* `--device=/dev/video0:/dev/video0` - adds a camera device to the container's context
* `-v $(pwd):/data` - mounts current (`kenning-ros2-demo`) directory in the `/data` directory in the container's context
* `-v /tmp/.X11-unix/:/tmp/.X11-unix/` - passes the X11 socket directory to the container's context (to allow running GUI application)
* `-e DISPLAY=$DISPLAY`, `-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR` - adds X11-related environment variables
* `--runtime nvidia` - adds GPUs to the container's context for computing and displaying purposes

Then, go to the workspace directory in the container:

```
cd /data
```
## Install Kenning and ONNXRUNTIME

Before starting the demo we need to install **Kenning**, start by creating virtual
environment:

```bash
python -m venv --system-site-packages .venv
source .venv/bin/activate
```

Then install **Kenning** with necessary dependencies:

You may update pip before processing, to avoid installation errors:

``` bash
pip install "./kenning[object_detection,pose_estimation]"
```

then install compatible **onnxruntime-gpu** that you can get with:

``` bash
wget https://dl.antmicro.com/kenning/packages/onnxruntime_gpu-1.23.0-cp312-cp312-linux_aarch64.whl
```
then install it:
``` bash
pip install ./onnxruntime_gpu-1.23.0-cp312-cp312-linux_aarch64.whl
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
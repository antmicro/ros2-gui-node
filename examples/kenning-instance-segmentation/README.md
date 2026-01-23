# Instance segmentation visualization with Kenning runtime

This demo runs an instance segmentation algorithm on frames from the camera and displays detected masks.

The demo consists of three nodes:

* Camera node - reads data from a camera and exposes its settings
* GUI node - renders a window with camera view, instance segmentation view, and additional logs
* Instance segmentation node - Kenning-based node reading frames from the Camera node, running YOLACT instance segmentation model, and sending predictions to the GUI node.

## Necessary dependencies

This demo requires:

* A camera for streaming frames
* A CUDA-enabled NVIDIA GPU for inference acceleration
* [repo tool](https://gerrit.googlesource.com/git-repo/+/refs/heads/main/README.md) to clone all necessary repositories
* [Docker](https://www.docker.com/) to use a prepared environment
* [nvidia-container-toolkit](https://github.com/nvidia/nvidia-container-toolkit) to provide access to the GPU in the Docker container

All of the necessary build, runtime and development dependencies are provided in the [Dockerfile](./Dockerfile).
It contains:

* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) environment
* [OpenCV](https://github.com/opencv/opencv) for image processing
* [Apache TVM](https://github.com/apache/tvm) for model optimization and runtime
* Dependencies for the [Kenning framework](https://github.com/antmicro/kenning)
* CUDNN and CUDA libraries for faster acceleration on GPUs
* Additional development tools

It can be either pulled from the Docker registry using:

```
docker pull ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo
```

or you can build it from scrach, by cloning ros2-gui-node and executing:

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
repo init -u git@github.com:antmicro/ros2-gui-node.git -m examples/kenning-instance-segmentation/manifest.xml

repo sync -j`nproc`

mkdir build
```

> **NOTE**
>
> Before executing `repo` command you may need to set up git credential by typing into terminal:
>
> ``` bash
> git config --global user.email "<e-mail address>"
> git config --global user.name "Name Surname"
> ```

It downloads the following repositories:

* [Kenning](https://github.com/antmicro/kenning) for model optimization and runtime, in the `kenning` directory.
* [ROS 2 Camera node](https://github.com/antmicro/ros2-camera-node) for obtaining frames from the camera and serving its parameters as ROS 2 parameters, in the `src/camera_node` directory.
* [Kenning's ROS 2 messages and services](https://github.com/antmicro/ros2-kenning-computer-vision-msgs) for computer vision, in the `src/computer_vision_msgs` directory.
* This repository, in the `src/gui_node` directory.

## Starting the Docker environment

If you are using the Docker container, allow non-network local connections to X11 so that the GUI can be started from the Docker container:

```
xhost +local:
```

Then, run a Docker container under the `kenning-ros2-demo` directory with:

```
./src/gui_node/environments/run-docker.sh
```

`NOTE:` In case you have built the image manually, e.g. with name `kenning-ros2-demo`, run `DOCKER_IMAGE=kenning-ros2-demo ./run-docker.sh`.
Also, if you want to change the camera path, set the `CAMERA_PATH` variable with your desired path before running the script.

Scripts checks for the presence of NVIDIA drivers - if NVIDIA GPU is not present, the container will run in CPU-only mode.
If you want to explicitly run the container without GPU acceleration, set:

```
export USE_PLATFORM=cpu
```
before running the script.

This script starts the image with:

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

For example, for 530.41.03, install the following in the container:

```
apt-get update && apt-get install libnvidia-gl-530
```

Then, go to the workspace directory in the container:

```
cd /data
```

## Optimizing the YOLACT model with Kenning

To get the runtime to perform in real time, you need to first optimize YOLACT model for your machine.
In this example, we will use the Apache TVM compiler to compile the model for the GPU.

### Install TVM for CPU only inference

If you run CPU only inference you need to install:

```bash
pip install --force-reinstall apache-tvm
```

and then run:

```bash
ldconfig
```

First, install Kenning with its necessary dependencies:

```bash
pip install "./kenning[object_detection]"
```

Then, run the [scenario with YOLACT optimizations for the GPU](./examples/kenning-instance-segmentation/yolact-tvm-gpu-optimization.yaml).
It will:

* Load the `yolact.onnx` model, containing YOLACT implementation
* Compile the model using TVM for a CUDA-enabled GPU with support for CUDNN and CUBLAS libraries.

```bash
kenning optimize --cfg src/gui_node/examples/kenning-instance-segmentation/yolact-tvm-gpu-optimization.yaml
```

To evaluate execution of the model **without** GPU acceleration, run:

```bash
kenning optimize --cfg src/gui_node/examples/kenning-instance-segmentation/yolact-tvm-cpu-optimization.yaml
```

## Building GUI node and Camera node

First of all, load the `setup.sh` script for ROS 2 tools, e.g.:

```bash
source /opt/ros/setup.sh
```

Then, build the GUI node and the Camera node with:

```bash
colcon build --base-paths src --cmake-args -DBUILD_KENNING_YOLACT_DEMO=y
```

## Running the demo with a camera

Then, to run the demo, load the ROS 2 environment including the newly built packages:

```bash
source install/setup.sh
```

Next, launch Kenning, Camera node, and GUI node using the launch file [`kenning-instance-segmentation.py use_gui:=True`](./kenning-instance-segmentation.py):

```
ros2 launch gui_node kenning-instance-segmentation.py
```

In case you want to change the path to the camera, use the `camera_path:=<new-path>` argument, e.g.:

```
ros2 launch gui_node kenning-instance-segmentation.py camera_path:=/dev/video1
```

For CPU-only demo of the instance segmentation run [`kenning-instance-segmentation-cpu.py use_gui:=True`](./kenning-instance-segmentation-cpu.py) as follows:

```
ros2 launch gui_node kenning-instance-segmentation-cpu.py
```

To change camera path, use camera_path parameter as follows:

```
ros2 launch gui_node kenning-instance-segmentation-cpu.py camera_path:=/dev/video1
```

Lastly, a GUI should appear, with:

* Direct view from Camera node
* Instance segmentation view based on predictions from Kenning (started using `kenning flow` with [`./kenning-instance-segmentation.yaml`](./examples/kenning-instance-segmentation/kenning-instance-segmentation.yaml) or [`kenning-instance-segmentation-cpu.yaml`](./examples/kenning-instance-segmentation/kenning-instance-segmentation-cpu.yaml) if you don't use GPU )
* A widget visualizing a list of detected objects, with a possibility to filter out not interesting classes.

# Instance segmentation visualization with Kenning runtime

This demo runs an instance segmentation algorithm on frames from the camera and displays detected masks.

The demo consists of three nodes:

* Camera node - reads data from the camera and exposes its settings
* GUI node - renders a window with camera view, instance segmentation view, and additional logs
* Instance segmentation node - Kenning-based node reading frames from the Camera node, running YOLACT instance segmentation model, and sending predictions to the GUI node

## Necessary dependencies

This demo requires:

* A camera for streaming frames
* A [repo tool](https://gerrit.googlesource.com/git-repo/+/refs/heads/main/README.md) for cloning all of the necessary repositories
* [Docker](https://www.docker.com/) to use a prepared environment

All of the necessary build, runtime and development dependencies are provided in the [Dockerfile](./Dockerfile).
It contains:

* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) environment
* [OpenCV](https://github.com/opencv/opencv) for image processing
* [Apache TVM](https://github.com/apache/tvm) for model optimization and runtime
* Dependencies for the [Kenning framework](https://github.com/antmicro/kenning)
* CUDNN and CUDA libraries for faster acceleration on GPUs
* Development tools

It can be either pulled from Docker registry using:

```bash
docker pull ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo
```

or built from scratch with:

```bash
sudo docker build -t kenning-ros2-environment .
```

## Downloading the demo

First off, let's create a workspace directory, where downloaded repositories will be stored:

```bash
mkdir kenning-ros2-demo && cd kenning-ros2-demo
```

Then, download all of the dependencies using the `repo` tool:

```bash
repo init -u git@github.com:antmicro/ros2-gui-node.git -m examples/kenning-instance-segmentation/manifest.xml

repo sync -j12

repo forall git lfs pull
```

## Starting the Docker environment

In the beginning, if the Docker container is used, let's allow non-network local connections to X11 so that GUI can be started from the Docker container:

```bash
xhost +local:
```

Thirdly, let's run a Docker container with:

```bash
./run-docker.sh
```

`NOTE:` In case you have built the image manually, e.g. with name `kenning-ros2-environment`, run `DOCKER_IMAGE=kenning-ros2-environment ./run-docker.sh`.
Also, if you want to change the camera path, set `CAMERA_PATH` variable with desired path before running the script.

This script starts the  image with:

* `--device=/dev/video0:/dev/video0` - adds a camera device to container's context
* `-v $(pwd):/data` - mounts current (`kenning-ros2-demo`) directory under `/data` directory in the container's context
* `-v /tmp/.X11-unix/:/tmp/.X11-unix/` - passes directory with X11 socket to the container's context (to allow running GUI application)
* `-e DISPLAY=$DISPLAY`, `-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR` - adds X11-related environment variables
* `--gpus='all,"capabilities=compute,utility,graphics,display"'` - adds GPUs to the container's context, for computing and displaying purposes

Then, in the Docker container, you need to install graphics libraries for NVIDIA that match your host's drivers.
To check the NVIDIA drivers version, run:

```bash
nvidia-smi
```

And check the `Driver version`.

For example, for 530.41.03, install the following in the container:

```bash
apt-get update && apt-get install libnvidia-gl-530
```

Then, go to the workspace directory in the container:

```bash
cd /data
```

## Optimizing the YOLACT model with Kenning

To get real-time performance of the runtime, you need to first optimize YOLACT model for your machine.
In this example, let's use Apache TVM compiler to compile model for GPU.

First, let's install Kenning with its necessary dependencies:

```bash
pip install --no-deps kenning/
```

After this, let's run the [scenario with YOLACT optimizations for GPU](https://github.com/antmicro/kenning/blob/main/scripts/jsonconfigs/yolact-tvm-gpu-detection.json).
It will:

* Load the `yolact.onnx` model, containing YOLACT implementation
* Compile the model using TVM for CUDA-enabled GPU with support for CUDNN and CUBLAS libraries.

```bash
cd kenning/
kenning optimize  --json-cfg scripts/jsonconfigs/yolact-tvm-gpu-detection.json
```

## Building GUI node and Camera node

First of all, load the `setup.sh` script for ROS 2 tools, e.g.:

```bash
source /opt/ros/setup.sh
```

Then, build the GUI node and the Camera node with:

```bash
cd /data
colcon build --base-paths src --cmake-args -DBUILD_KENNING_YOLACT_DEMO=y
```

## Running the demo

Then, to run the demo, load the ROS 2 environment including the newly built packages:

```bash
source install/setup.sh
```

After this, launch Kenning, Camera node, and GUI node using the launch file `kenning-instance-segmentation.py`:

```bash
ros2 launch gui_node kenning-instance-segmentation.py
```

In case you want to change the path to the camera, use `camera_path:=<new-path>` argument, e.g.:

```bash
ros2 launch gui_node kenning-instance-segmentation.py camera_path:=/dev/video1
```

Finally, a GUI should appear, with:

* Direct view from Camera node
* Instance segmentation view based on predictions from Kenning
* A widget visualizing a list of detected objects, with a possibility to filter out not interesting classes.

# Instance segmentation visualization with Kenning runtime

This demo runs an instance segmentation algorithm on frames from the camera and displays detected masks.

The demo consists of three nodes:

* Camera node - reads data from the camera and exposes its settings
* GUI node - renders a window with camera view, instance segmentation view, and additional logs
* Instance segmentation node - Kenning-based node reading frames from the Camera node, running YOLACT instance segmentation model, and sending predictions to the GUI node

This demo requires:

* Connected camera
* ROS 2 environment installed in the system
* `repo` tool for obtaining necessary repositories

## Preparing the Docker environment

Necessary environment to run the ROS 2 demo can be found in the [Dockerfile](./Dockerfile).
It provides all of the necessary libraries to get started with Kenning and ROS 2 runtime and compilation (with some additional helper tools for faster development).

To build the Docker image, run:

```bash
sudo docker build -t kenning-ros2-environment .
```

It will build a CUDA-enabled image with:

* ROS 2
* OpenCV
* Apache TVM framework for model optimization and runtime
* Dependencies for compiling and running the YOLACT model
* CUDNN for faster acceleration on GPUs

## Starting the Docker environment

First of, let's create a workspace directory which will store persistent files with the repositories and binaries:

```bash
mkdir kenning-ros2-demo
cd kenning-ros2-demo
```

Secondly, let's allow non-network local connections to X11 so that GUI can be started from Docker container:

```bash
xhost +local:
```

Thirdly, let's run a Docker container with:

```bash
docker run -it \
    --device=/dev/video0:/dev/video0 \
    -v $(pwd):/data \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --gpus='all,"capabilities=compute,utility,graphics,display"' \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    kenning-ros2-environment \
    /bin/bash
```

This command starts a `kenning-ros2-environment` image with:

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

## Downloading the demo

Download all of the dependencies using the `repo` tool:

```bash
repo init -u git@github.com:antmicro/ros2-gui-node.git -m examples/kenning-instance-segmentation/manifest.xml

repo sync -j12

repo forall git lfs pull
```

## Optimizing the YOLACT model with Kenning

To get real-time performance of the runtime, you need to first optimize YOLACT model for your machine.
In this example, let's use Apache TVM compiler to compile model for GPU.

Since the container provides all of the necessary dependencies, you can only add the cloned `/data/kenning` repository to the `PYTHONPATH` variable

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
kenning optimize  --json-cfg scripts/jsonconfigs/yolact-tvm-gpu-detection.json --verbosity INFO
```

## Building GUI node and Camera node

First of all, load the `setup.sh` script for ROS 2 tools, e.g.:

```bash
source /opt/ros/setup.sh
```

Then, build the GUI node and the Camera node with:

```bash
cd /data
colcon build --base-paths src --cmake-args -DBUILD_YOLACT_DEMO=y
```

## Running the demo

Then, to run the demo, load the ROS 2 environment including the newly built packages:

```bash
source install/setup.sh
```

And then launch Kenning, Camera node, and GUI node using the launch file `kenning-instance-segmentation.py`:

```bash
ros2 launch gui_node kenning-instance-segmentation.py
```

Finally, a GUI should appear, with:

* Direct view from Camera node
* Instance segmentation view based on predictions from Kenning
* A widget visualizing a list of detected objects, with a possibility to filter out not interesting classes.

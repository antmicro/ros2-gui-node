#!/bin/bash

# This script runs the Docker container for ROS 2 and Kenning integration

docker run -it \
    --device=/dev/video0:/dev/video0 \
    -v $(pwd):/data \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --gpus='all,"capabilities=compute,utility,graphics,display"' \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo \
    /bin/bash

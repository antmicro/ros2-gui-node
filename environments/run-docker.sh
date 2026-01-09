#!/bin/bash

# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script runs the Docker container for ROS 2 and Kenning integration

if [[ $1 = "help" ]]; then
    echo "Example of instance segmentation using Kenning and ROS 2"
    echo 
    echo "It utillizes NVIDIA GPU for faster inference but CPU can be used as well"
    echo "In order to run example explicitly with CPU inference run:"
    echo 
    echo "  run-docker.sh cpu"
    echo 
    exit
fi

CAMERA_PATH=${CAMERA_PATH:-/dev/video0}
DOCKER_IMAGE=${DOCKER_IMAGE:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

GPU_PARAMS=--gpus='all,"capabilities=compute,utility,graphics,display"'

USE_CPU=""

if ! command -v nvidia-smi >/dev/null 2>&1
then
    echo "Nvidia GPU not found, using CPU"
    GPU_PARAMS=""
fi

if [[ $1 = "cpu" ]]; then
    echo "Selected to run example on cpu"
    GPU_PARAMS=""    
fi

docker run -it \
    --privileged \
    --device=$CAMERA_PATH:$CAMERA_PATH \
    -v $(pwd):/data \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    $GPU_PARAMS \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    $DOCKER_IMAGE \
    /bin/bash -c "$2"

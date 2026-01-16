#!/bin/bash

# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script runs the Docker container for ROS 2 and Kenning integration

CAMERA_PATH=${CAMERA_PATH:-/dev/video0}
DOCKER_IMAGE=${DOCKER_IMAGE:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

GPU_PARAMS=--gpus='all,"capabilities=compute,utility,graphics,display"'

USE_PLATFORM=${USE_PLATFORM:""}

INTERACTIVE_MODE="-it"

if ! command -v nvidia-smi >/dev/null 2>&1
then
    echo "Nvidia GPU not found, using CPU"
    USE_PLATFORM="cpu"
fi

if [[ $USE_PLATFORM = "cpu" ]]; then
    echo "Selected to run example on cpu"
    GPU_PARAMS=""
elif [[ $USE_PLATFORM = "jetson" ]]; then
    echo "Selected Nvidia Jetson platform"
    GPU_PARAMS="--runtime nvidia"
fi

if [ -n "$USE_NON_INTERACTIVE" ]; then
    INTERACTIVE_MODE=""
fi

docker run --privileged \
    $INTERACTIVE_MODE \
    --device=$CAMERA_PATH:$CAMERA_PATH \
    -v $(pwd):/data \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    $GPU_PARAMS \
    -e KENNING_CACHE_DIR="/data/kenning_cache" \
    -e USE_PLATFORM=$USE_PLATFORM \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    $DOCKER_IMAGE \
    $1
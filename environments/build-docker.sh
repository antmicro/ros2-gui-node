#!/bin/bash

# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script builds the Docker container for ROS 2 and Kenning integration

DOCKER_TAG=${DOCKER_TAG:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

BASE_IMAGE="nvidia/cuda:12.9.1-cudnn-devel-ubuntu24.04"
BUILD_TVM="y"

SCRIPTDIR=$(dirname "$(realpath "$0")")

pushd "${SCRIPTDIR}" || exit
if [[ $1 = "jetson" ]]; then
    jetson-containers build cudastack:standard

    BASE_IMAGE="$(autotag cudastack:standard)"
    BUILD_TVM=""
    ADDITIONAL_PACKAGES="kmod \
    vulkan-validationlayers" 
fi

mkdir -p third-party/

if [ ! -d "third-party/tvm" ]
then
    git clone --recursive https://github.com/apache/tvm.git --depth 1 --branch v0.12.0 third-party/tvm
    git clone --recursive https://github.com/ros-perception/vision_opencv --branch humble third-party/vision_opencv
fi

docker build . --tag "${DOCKER_TAG}" --build-arg BASE_IMAGE="$BASE_IMAGE" --build-arg BUILD_TVM="$BUILD_TVM"

popd || exit

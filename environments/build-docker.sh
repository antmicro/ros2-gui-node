#!/bin/bash

# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script builds the Docker container for ROS 2 and Kenning integration

DOCKER_TAG=${DOCKER_TAG:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

SCRIPTDIR=$(dirname "$(realpath "$0")")

pushd "${SCRIPTDIR}" || exit

mkdir -p third-party/

if [ ! -d "third-party/tvm" ]
then
    git clone --recursive https://github.com/apache/tvm.git --depth 1 --branch v0.12.0 third-party/tvm
    git clone --recursive https://github.com/ros-perception/vision_opencv --branch humble third-party/vision_opencv
fi

docker build . --tag "${DOCKER_TAG}"

popd || exit

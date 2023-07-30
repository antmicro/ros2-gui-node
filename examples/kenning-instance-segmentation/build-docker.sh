#!/bin/bash

# Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script builds the Docker container for ROS 2 and Kenning integration

DOCKER_TAG=${DOCKER_TAG:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

SCRIPTDIR=$(dirname $(realpath $0))

pushd ${SCRIPTDIR}

mkdir -p third-party/

if [ ! -d "third-party/tvm" ]
then
    git clone --recursive https://github.com/apache/tvm.git --depth 1 --branch v0.12.0 third-party/tvm
fi

docker build . --tag ${DOCKER_TAG}

popd

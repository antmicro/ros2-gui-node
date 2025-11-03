#!/bin/bash

# Copyright (c) 2022-2025 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script builds the Docker container for ROS 2 and Kenning integration

DOCKER_TAG=${DOCKER_TAG:-ghcr.io/antmicro/ros2-gui-node:kenning-ros2-demo}

SCRIPTDIR=$(dirname "$(realpath "$0")")

pushd "${SCRIPTDIR}" || exit

docker build . --tag "${DOCKER_TAG}"

popd || exit

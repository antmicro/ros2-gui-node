#!/bin/bash

# Copyright (c) 2026 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# This script executes demo instructions step by step

set -ex

KENNING_MULTIMODEL_DEMO_PATH=${KENNING_MULTIMODEL_DEMO_PATH:-"."}

function outside_docker() {
    # Share X-window connection
    xhost +local:

    pushd $KENNING_MULTIMODEL_DEMO_PATH
    # Run container
    ./src/gui_node/environments/run-docker.sh "/data/src/gui_node/examples/kenning-multimodel-demo/tools/general/run-demo.sh docker"

    popd
}

function inside_docker() {
    # Go to demo directory
    cd /data

    # Create and source virtual envrionemnt
    source .venv/bin/activate

    # Source ROS 2
    source /opt/ros/setup.sh

    # Source demo packages
    source install/setup.sh

    # Configure camera
    install/camera_node/bin/grabthecam-demo -c /dev/video0 -d 1280,720

    # Start the demo
    ros2 launch gui_node kenning-multimodel-demo.py use_gui:=True
}

if [[ $1 = "docker" ]]; then
    inside_docker
else
    outside_docker
fi

#!/bin/bash

# Copyright (c) 2026 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# A script for initializing kenning multimodel demo

set -ex

function outside_docker() {
    # Create directory for demo
    mkdir -p kenning-ros2-demo

    pushd kenning-ros2-demo

    # Prepare repositories
    repo init -u https://github.com/antmicro/ros2-gui-node.git -m examples/kenning-multimodel-demo/manifest.xml

    repo sync -j`nproc`

    # Set a Kenning multimodel demo path location
    export KENNING_MULTIMODEL_DEMO_PATH=$(realpath .)

    # Execute demo
    ./src/gui_node/environments/run-docker.sh "/data/src/gui_node/examples/kenning-multimodel-demo/tools/general/init-demo.sh docker"

    popd

}

function inside_docker() {
    # Go to demo directory
    cd /data

    # Create and source virtual envrionemnt
    python -m venv --system-site-packages .venv
    source .venv/bin/activate

    # Install kenning with necessary requirments
    pip install "./kenning[object_detection,pose_estimation]"

    if [[ $USE_PLATFORM = "jetson" ]]; then
         # Download onnxruntime_gpu for Jetson
        wget -N https://dl.antmicro.com/kenning/packages/onnxruntime_gpu-1.23.0-cp312-cp312-linux_aarch64.whl
        # Install onnx runtime
        pip install ./onnxruntime_gpu-1.23.0-cp312-cp312-linux_aarch64.whl
    fi

    # Prefetch required resources
    python -m kenning download-resources --cfg /data/src/gui_node/examples/kenning-multimodel-demo/*.yaml

    # Source ROS 2
    source /opt/ros/setup.sh

    # Build demo
    colcon build --base-paths src --cmake-args -DBUILD_KENNING_MULTIMODEL_DEMO=y
}

if [[ $1 = "docker" ]]; then
    inside_docker
else
    # You can specified platform with first argument
    export USE_PLATFORM="$1"
    outside_docker
fi

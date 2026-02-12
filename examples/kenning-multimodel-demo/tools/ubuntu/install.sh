#!/bin/bash

# Copyright (c) 2026 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

# A script used to install service for starting Kenning Multimodel Demo

set -ex

if [[ -z $KENNING_MULTIMODEL_DEMO_PATH ]]; then
    echo "Demo path not specified, searching a path."

    for path in $(find / -name kenning-ros2-demo 2>/dev/null); do
        check_path="$path/src/gui_node/examples/kenning-multimodel-demo/tools/general/run-demo.sh"

        if [[ -n $(ls $check_path 2>/dev/null) ]]; then
            KENNING_MULTIMODEL_DEMO_PATH=$path
            break
        fi
    done

    if [[ -z $KENNING_MULTIMODEL_DEMO_PATH ]]; then
        echo "No demo is installed on the system"
        exit
    fi
fi

RUN_DEMO_SCRIPT_DIR=${INSTALL_SCRIPT_DIR%/*/*}/tools/general

RUN_DEMO_SCRIPT="$RUN_DEMO_SCRIPT_DIR/run-demo.sh"
USER=$USER
USE_PLATFORM=${USE_PLATFORM:-"$1"}

(DEMO_PATH="$RUN_DEMO_SCRIPT" USER="$USER" \
USE_PLATFORM="$USE_PLATFORM" KENNING_MULTIMODEL_DEMO_PATH="$KENNING_MULTIMODEL_DEMO_PATH" \
envsubst < $INSTALL_SCRIPT_DIR/kenning.multimodel.demo.service.template) > "/tmp/kenning.multimodel.demo.service"

sudo mv /tmp/kenning.multimodel.demo.service /etc/systemd/system/kenning.multimodel.demo.service

sudo systemctl enable kenning.multimodel.demo.service

echo $RUN_DEMO_SCRIPT_DIR
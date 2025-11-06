# Copyright (c) 2022-2025 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import ExecuteProcess

def generate_launch_description():
    camera_path = DeclareLaunchArgument(
        "camera_path",
        default_value="/dev/video0",
        description="Path to camera device"
    )

    use_gui = LaunchConfiguration('use_gui')
    start_nvidia_mps = LaunchConfiguration('start_nvidia_mps')

    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="False",
        description="Set to true if you want to use GUI"
    )

    start_nvidia_mps_arg = DeclareLaunchArgument(
        "start_nvidia_mps",
        default_value="False",
        description="Set to true to start nvidia-cuda-mps-control"
    )

    camera_node_container = ComposableNodeContainer(
        name='camera_node_container',
        namespace='camera_node',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_node',
                plugin='camera_node::CameraNode',
                name='camera_node',
                parameters=[
                    {'camera_path': LaunchConfiguration("camera_path")}
                ]
            ),
        ],
        output='both',
    )

    gui_node = ComposableNodeContainer(
        name='gui_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::KenningMultiModelGuiComponent',
                name='sample_gui_node')
        ],
        output='both',
        on_exit=launch.actions.Shutdown(),
        condition=IfCondition(use_gui)
    )

    kenning_yolact_node = Node(
        name="kenning_yolact_node",
        executable="kenning",
        arguments=["ros","flow","--verbosity","INFO"],
        parameters=[{
            "config_file":"./src/gui_node/examples/kenning-multimodel-demo/kenning-instance-segmentation.yaml"
        }]
    )

    kenning_mmpose_node = Node(
        name="kenning_mmpose_node",
        executable="kenning",
        arguments=["ros","flow","--verbosity","DEBUG"],
        parameters=[{
            "config_file":"./src/gui_node/examples/kenning-multimodel-demo/kenning-pose-estimation.yaml"
        }]
    )

    kenning_dinov2_node = Node(
        name="kenning_dinov2_node",
        executable="kenning",
        arguments=["ros","flow","--verbosity","DEBUG"],
        parameters=[{
            "config_file":"./src/gui_node/examples/kenning-multimodel-demo/kenning-depth-estimation.yaml"
        }]
    )
    nvidia_mps_node = ExecuteProcess(
        name="cuda_mps_node",
        cmd=["nvidia-cuda-mps-control", "-f"],
        additional_env={
            "CUDA_MPS_PIPE_DIRECTORY": "/tmp/nvidia-mps",
            "CUDA_MPS_LOG_DIRECTORY": "/tmp/mps-logs",
        },
        output='both',
        on_exit=launch.actions.Shutdown(),
        condition=IfCondition(start_nvidia_mps)
    )

    return launch.LaunchDescription([
        start_nvidia_mps_arg,
        SetEnvironmentVariable("CUDA_MPS_PIPE_DIRECTORY", os.getenv("CUDA_MPS_PIPE_DIRECTORY","/tmp/nvidia-mps")),
        SetEnvironmentVariable("CUDA_MPS_LOG_DIRECTORY", os.getenv("CUDA_MPS_LOG_DIRECTORY","/tmp/mps-logs")),
        nvidia_mps_node,
        use_gui_arg,
        start_nvidia_mps_arg,
        camera_path,
        camera_node_container,
        gui_node,
        kenning_yolact_node,
        kenning_mmpose_node,
        kenning_dinov2_node
    ])

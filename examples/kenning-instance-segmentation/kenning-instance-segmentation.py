# Copyright (c) 2025 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0


import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable

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
                plugin='gui_node::KenningYolactGuiComponent',
                name='sample_gui_node')
        ],
        output='both',
        on_exit=launch.actions.Shutdown(),
        condition=IfCondition(use_gui)
    )

    kenning_node = Node(
        name="kenning_node",
        executable="kenning",
        arguments=["ros","flow","--verbosity","DEBUG"],
        parameters=[{
            "config_file":"./src/gui_node/examples/kenning-instance-segmentation/kenning-instance-segmentation.json"
        }],
        on_exit=launch.actions.Shutdown()
    )

    nvidia_mps_node = ExecuteProcess(
        name="cuda_mps_node",
        cmd=["nvidia-cuda-mps-control", "-f"],
        additional_env={
            "CUDA_MPS_PIPE_DIRECTORY": "/tmp/nvidia-mps",
            "CUDA_MPS_LOG_DIRECTORY": "/tmp/mps-logs",
        },
        on_exit=launch.actions.Shutdown(),
        condition=IfCondition(start_nvidia_mps)
    )

    return launch.LaunchDescription([
        SetEnvironmentVariable("CUDA_MPS_PIPE_DIRECTORY", "/tmp/nvidia-mps",
                               condition=IfCondition(start_nvidia_mps)),
        SetEnvironmentVariable("CUDA_MPS_LOG_DIRECTORY", "/tmp/mps-logs",
                               condition=IfCondition(start_nvidia_mps)),
        nvidia_mps_node,
        start_nvidia_mps_arg,
        use_gui_arg,
        camera_path,
        camera_node_container,
        gui_node,
        kenning_node,
    ])

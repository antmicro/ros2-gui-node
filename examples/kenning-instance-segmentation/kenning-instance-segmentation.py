
# Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0


import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_path = DeclareLaunchArgument(
        "camera_path",
        default_value="/dev/video0",
        description="Path to camera device"
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
                plugin='gui_node::YolactGuiComponent',
                name='sample_gui_node')
        ],
        output='both',
        on_exit=launch.actions.Shutdown()
    )

    kenning_node = ExecuteProcess(
        name="kenning_node",
        cmd='kenning flow --json-cfg ./scripts/jsonflowconfigs/ros2/yolact-ros2-realtime-segmentation.json'.split(' '),  # noqa: E501
        cwd="./kenning"
    )

    return launch.LaunchDescription([
        camera_path,
        camera_node_container,
        gui_node,
        kenning_node,
    ])

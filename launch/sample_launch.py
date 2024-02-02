# Copyright (c) 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    sample_gui_node = ComposableNodeContainer(
        name='sample_gui_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::SampleGuiComponent',
                name='sample_gui_node')
        ],
        output='both',
        on_exit=launch.actions.Shutdown(),
    )

    sample_publish_node = ComposableNodeContainer(
        name='sample_publish_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::SamplePublisherComponent',
                name='sample_publish_node')
        ],
        output='both',
    )
    return launch.LaunchDescription([sample_gui_node, sample_publish_node])

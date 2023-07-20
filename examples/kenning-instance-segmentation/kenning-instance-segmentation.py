import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess


def generate_launch_description():
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
                parameters=[{
                    'camera_path': '/dev/video0'
                }]),
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
        camera_node_container,
        gui_node,
        kenning_node,
    ])

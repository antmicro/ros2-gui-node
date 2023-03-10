import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    topic_publisher = ComposableNodeContainer(
        name='time_publisher_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::TimePublisher',
                name='time_publisher')
        ],
        output='both',
    )

    gui_topic_subscriber = ComposableNodeContainer(
        name='gui_subscriber_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::TimeSubscriber',
                name='gui_subscriber')
        ],
        output='both',
    )

    rosout_topic_subscriber = ComposableNodeContainer(
        name='rosout_subscriber_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::RosoutSubscriberNode',
                name='rosout_subscriber')
        ],
        output='both',
    )

    video_publisher = ComposableNodeContainer(
        name='video_publisher_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gui_node',
                plugin='gui_node::VideoPublisher',
                name='video_publisher')
        ],
        output='both',
    )

    return launch.LaunchDescription([rosout_topic_subscriber, video_publisher])

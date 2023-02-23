#ifndef GUI_NODE_ROS_DATA_ROS_SUBSCRIBER_DATA_HPP
#define GUI_NODE_ROS_DATA_ROS_SUBSCRIBER_DATA_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporing subscribing to a ROS2 topic and
 * processing received messages.
 *
 * @tparam Tmsg  ROS2 message type.
 * @tparam Tdata Data type to be used for processing.
 */
template <class Tmsg, class Tdata> class RosSubscriberData : public RosData
{
private:
    /**
     * Callback function for subscriber
     *
     * @param msg Message to convert to data
     */
    void callback(const typename Tmsg::SharedPtr &msg) { Tdata data = data_function(msg); }

    typename rclcpp::Subscription<Tmsg>::SharedPtr subscriber;          ///< Subscriber
    std::function<Tdata(const typename Tmsg::SharedPtr)> data_function; ///< Function to convert message to data
public:
    /**
     * Constructor
     *
     * @param node The node to create the subscriber on
     * @param topic The name of the topic to subscribe to
     * @param data_function Function to convert message to data
     * @param queue_size The size of the message queue
     */
    RosSubscriberData(std::shared_ptr<GuiNode> node, const std::string &topic,
                      std::function<Tdata(const typename Tmsg::SharedPtr)> data_function, size_t queue_size = 10)
        : RosData(node), data_function(data_function)
    {
        subscriber = node->create_subscription<Tmsg>(
            topic, queue_size, [this](const typename Tmsg::SharedPtr msg) -> void { this->callback(msg); });
    }
};

} // namespace gui_node
#endif // GUI_NODE_ROS_DATA_ROS_SUBSCRIBER_DATA_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporting subscribing to a ROS2 topic and
 * processing received messages.
 *
 * @tparam Tmsg ROS2 message type.
 * @tparam Tdata Data type to be used for processing.
 */
template <class Tmsg, class Tdata>
class RosSubscriberData : public RosData
{
private:
    /**
     * Callback function for subscriber.
     *
     * @param msg Message to convert to data.
     */
    void callback(const typename Tmsg::SharedPtr &msg)
    {
        data = data_function(msg);
        data_changed = true;
    }

    typename rclcpp::Subscription<Tmsg>::SharedPtr subscriber;          ///< Subscriber
    std::function<Tdata(const typename Tmsg::SharedPtr)> data_function; ///< Function to convert message to data
    Tdata data;                                                         ///< Data
public:
    /**
     * Constructor.
     *
     * @param node The node to create the subscriber on.
     * @param topic The name of the topic to subscribe to.
     * @param data_function Function to convert message to data.
     * @param queue_size The size of the message queue.
     */
    RosSubscriberData(
        std::shared_ptr<GuiNode> node,
        const std::string &topic,
        std::function<Tdata(const typename Tmsg::SharedPtr)> data_function,
        size_t queue_size = 10)
        : RosData(node), data_function(data_function)
    {
        subscriber = node->create_subscription<Tmsg>(
            topic,
            queue_size,
            [this](const typename Tmsg::SharedPtr msg) -> void { this->callback(msg); });
    }

    /**
     * Get the last received data.
     *
     * @return The last received data.
     */
    Tdata getData()
    {
        data_changed = false;
        return data;
    }
};

} // namespace gui_node

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporting publishing of ROS2 messages to the specific topic.
 *
 * @tparam Tmsg Type of the ROS2 service message.
 * @tparam Tdata Type of the data to be processed and published.
 */
template <class Tmsg, class Tdata>
class RosPublisherData : public RosData
{
private:
    typename rclcpp::Publisher<Tmsg>::SharedPtr publisher; ///< Publisher
    std::function<Tmsg(const Tdata &)> msg_function;       ///< Function to convert data to message
    Tdata data;                                            ///< Last published data

public:
    /**
     * Constructor.
     *
     * @param node The node to create publisher on.
     * @param topic_name Name of the topic to publish to.
     * @param msg_function Function to convert data to message.
     * @param queue_size Size of the queue for messages.
     */
    RosPublisherData(
        std::shared_ptr<GuiNode> node,
        const std::string &topic_name,
        std::function<Tmsg(const Tdata &)> msg_function,
        size_t queue_size = 10)
        : RosData(node), msg_function(msg_function)
    {
        publisher = node->create_publisher<Tmsg>(topic_name, queue_size);
    }

    /**
     * Processes and publishes given data to topic.
     *
     * @param data Data to publish.
     */
    void publish(const Tdata &data_to_publish)
    {
        data = data_to_publish;
        data_changed = true;
        auto msg = msg_function(data);
        publisher->publish(msg);
    }

    /**
     * Gets last published data.
     *
     * @return Last published data.
     */
    Tdata getData()
    {
        data_changed = false;
        return data;
    }
};

} // namespace gui_node

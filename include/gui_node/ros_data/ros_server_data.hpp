#ifndef GUI_NODE_ROS_DATA_ROS_SERVER_DATA_HPP
#define GUI_NODE_ROS_DATA_ROS_SERVER_DATA_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporting ROS2 service server nodes.
 *
 * @tparam Tmsg Type of ROS2 service message.
 */
template <class Tmsg> class RosServiceServerData : public RosData
{
private:
    /**
     * Callback function for the ROS2 service server.
     * Processes the request and sends the response to the client.
     *
     * @param request The request message.
     * @param response The response message.
     */
    void callback(typename Tmsg::Request::SharedPtr request, typename Tmsg::Response::SharedPtr response)
    {
        service_function(request, response);
    }

    typename rclcpp::Service<Tmsg>::SharedPtr service; ///< The ROS2 service server.
    std::function<void(typename Tmsg::Request::SharedPtr, typename Tmsg::Response::SharedPtr)>
        service_function; ///< The service function to process requests.
public:
    /**
     * Constructor.
     *
     * @param node The node to create the service server on.
     * @param service_name The name of the ROS2 service.
     * @param service_function The service function to process requests.
     */
    RosServiceServerData(
        std::shared_ptr<GuiNode> node, const std::string &service_name,
        std::function<void(typename Tmsg::Request::SharedPtr, typename Tmsg::Response::SharedPtr)> service_function)
        : RosData(node), service_function(service_function)
    {
        service = node->create_service<Tmsg>(
            service_name,
            [this](typename Tmsg::Request::SharedPtr request, typename Tmsg::Response::SharedPtr response) -> void
            { callback(request, response); });
    }
};

};     // namespace gui_node
#endif // GUI_NODE_ROS_DATA_ROS_SERVER_DATA_HPP

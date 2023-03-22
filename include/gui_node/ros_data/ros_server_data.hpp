#pragma once

#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporting ROS2 service server nodes.
 *
 * @tparam Tmsg Type of ROS2 service message.
 * @tparam Tdata Type of data to be stored in the class.
 */
template <class Tmsg, class Tdata> class RosServiceServerData : public RosData
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
        data = service_function(request, response);
        data_changed = true;
    }

    typename rclcpp::Service<Tmsg>::SharedPtr service; ///< The ROS2 service server.
    std::function<void(typename Tmsg::Request::SharedPtr, typename Tmsg::Response::SharedPtr)>
        service_function; ///< The service function to process requests.
    Tdata data;           ///< The data from the service request.
public:
    /**
     * Constructor.
     *
     * @param node The node to create the service server on.
     * @param service_name The name of the ROS2 service.
     * @param service_function The service function to process requests, must return data from the request.
     */
    RosServiceServerData(
        std::shared_ptr<GuiNode> node, const std::string &service_name,
        std::function<Tdata(typename Tmsg::Request::SharedPtr, typename Tmsg::Response::SharedPtr)> service_function)
        : RosData(node), service_function(service_function)
    {
        service = node->create_service<Tmsg>(
            service_name,
            [this](typename Tmsg::Request::SharedPtr request, typename Tmsg::Response::SharedPtr response) -> void
            { callback(request, response); });
    }

    /**
     * Get the data from the service request.
     *
     * @return The data from the service request.
     */
    Tdata getData()
    {
        data_changed = false;
        return data;
    }
};

}; // namespace gui_node

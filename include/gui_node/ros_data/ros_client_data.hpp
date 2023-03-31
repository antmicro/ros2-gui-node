#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "gui_node/gui_node.hpp"

namespace gui_node
{

/**
 * Data class supporting sending requests to ROS2 services.
 *
 * @tparam Tmsg Type of ROS2 request.
 * @tparam Tdata Type of data to be requested.
 */
template <class Tmsg, class Tdata> class RosServiceClientData : public RosData
{
private:
    /**
     * Callback function for the service response.
     *
     * @param future Future of the service response.
     */
    void process_response(std::shared_future<std::shared_ptr<typename Tmsg::Response>> future)
    {
        auto result = future.get();
        if (result != nullptr)
        {
            data = *callback_function(result);
            data_changed = true;
        }
    }

    typename rclcpp::Client<Tmsg>::SharedPtr client; ///< Client of the service
    std::function<typename std::shared_ptr<Tdata>(typename std::shared_ptr<typename Tmsg::Response>)>
        callback_function; ///< Processes the response from the service
    Tdata data;            ///< Last received data

public:
    /**
     * Constructor.
     *
     * @param node The node to create the client on.
     * @param service_name The name of the service.
     * @param callback_function Function to process the response and return the Tdata.
     */
    RosServiceClientData(
        std::shared_ptr<GuiNode> node, std::string service_name,
        std::function<typename std::shared_ptr<Tdata>(typename std::shared_ptr<typename Tmsg::Response>)>
            callback_function)
        : RosData(node), callback_function(callback_function)
    {
        client = node->create_client<Tmsg>(service_name);
    }

    /**
     * Sends a request to the ROS2 service server.
     *
     * @param request The request to send.
     */
    void sendRequest(typename Tmsg::Request::SharedPtr request)
    {
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
        }
        auto future_result = client->async_send_request(
            request, std::bind(&RosServiceClientData::process_response, this, std::placeholders::_1));
    }

    /**
     * Returns the last received data.
     *
     * @return The last received data.
     */
    Tdata getData()
    {
        data_changed = false;
        return data;
    }

    /**
     * Returns whether the service is available.
     *
     * @return bool True if the service is available.
     */
    bool isServiceAvailable() { return client->service_is_ready(); }
};

} // namespace gui_node

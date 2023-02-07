#ifndef ROS_CLIENT_DATA_HPP_
#define ROS_CLIENT_DATA_HPP_

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
    typename rclcpp::Client<Tmsg>::SharedPtr client; ///< Client of the service
    std::function<typename std::shared_ptr<Tdata>(typename std::shared_ptr<typename Tmsg::Response>)>
        callback_function; ///< Proccesses the response from the service

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
            // TODO: Cache the data
            auto data = callback_function(result);
        }
    }

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
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }
        auto future_result = client->async_send_request(
            request, std::bind(&RosServiceClientData::process_response, this, std::placeholders::_1));
    }
};

};     // namespace gui_node
#endif // ROS_CLIENT_DATA_HPP_

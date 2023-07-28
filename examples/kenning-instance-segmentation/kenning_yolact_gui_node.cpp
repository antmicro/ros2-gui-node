/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kenning_computer_vision_msgs/msg/box_msg.hpp>
#include <kenning_computer_vision_msgs/msg/segmentation_msg.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/utils/detection.hpp"
#include "gui_node/widget/widget_detection.hpp"
#include "gui_node/widget/widget_rosout.hpp"
#include "gui_node/widget/widget_video.hpp"
#include "gui_node/widget/widget_control.hpp"

namespace gui_node
{

using RosSegmentationSubscriberData =
    RosSubscriberData<kenning_computer_vision_msgs::msg::SegmentationMsg, kenning_computer_vision_msgs::msg::SegmentationMsg::SharedPtr>;
using MsgImageSharedPtr = sensor_msgs::msg::Image::SharedPtr;
using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
using MsgRosoutSharedPtr = rcl_interfaces::msg::Log::SharedPtr;
using RosRosoutSubscriberData = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;

class KenningYolactGuiComponent
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

    /**
     * Prepares the display for the Kenning's instance segmentation message.
     *
     * @param instance_segmentation_msg The segmentation message.
     * @param bounding_boxes The vector of bounding boxes to fill.
     * @param filterclass The class to filter masks by.
     * @param threshold The threshold to filter masks by.
     * @return sensor_msgs::msg::Image The image to display.
     */
    sensor_msgs::msg::Image prep_display(
        kenning_computer_vision_msgs::msg::SegmentationMsg::SharedPtr instance_segmentation_msg,
        std::vector<BoundingBox> &bounding_boxes,
        const std::string &filterclass,
        const float &threshold)
    {
        kenning_computer_vision_msgs::msg::BoxMsg box;
        float score;
        int color_idx;
        std::string class_name;

        int height = instance_segmentation_msg->frame.height;
        int width = instance_segmentation_msg->frame.width;

        cv::Mat frame(height, width, CV_8UC3, instance_segmentation_msg->frame.data.data());

        for (size_t i = 0; i < instance_segmentation_msg->boxes.size(); i++)
        {
            class_name = instance_segmentation_msg->classes[i];
            score = instance_segmentation_msg->scores[i] * 100;
            color_idx = i % im_colors.size();
            box = instance_segmentation_msg->boxes[i];
            bounding_boxes.push_back(
                BoundingBox(box.xmin, box.ymin, box.xmax, box.ymax, class_name, score, im_colors[color_idx]));

            std::transform(class_name.begin(), class_name.end(), class_name.begin(), ::tolower);
            if (score >= threshold && (filterclass.empty() || class_name.find(filterclass) != std::string::npos))
            {
                // Apply mask
                cv::Mat mask(
                    instance_segmentation_msg->masks[i].dimension[0],
                    instance_segmentation_msg->masks[i].dimension[1],
                    CV_8UC1,
                    instance_segmentation_msg->masks[i].data.data());
                cv::resize(mask, mask, cv::Size(width, height));
                cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
                mask.setTo(colors[color_idx], mask);
                cv::addWeighted(mask, 0.4, frame, 1.0, 0, frame);
            }
        }
        return instance_segmentation_msg->frame;
    }

    /// ImGui color definitions
    std::vector<ImColor> im_colors{
        ImColor(ImVec4(0.96f, 0.26f, 0.21f, 1.0f)),
        ImColor(ImVec4(0.91f, 0.12f, 0.39f, 1.0f)),
        ImColor(ImVec4(0.61f, 0.15f, 0.69f, 1.0f)),
        ImColor(ImVec4(0.4f, 0.23f, 0.72f, 1.0f)),
        ImColor(ImVec4(0.25f, 0.32f, 0.71f, 1.0f)),
        ImColor(ImVec4(0.13f, 0.59f, 0.95f, 1.0f)),
        ImColor(ImVec4(0.01f, 0.66f, 0.96f, 1.0f)),
        ImColor(ImVec4(0.0f, 0.74f, 0.83f, 1.0f)),
        ImColor(ImVec4(0.0f, 0.59f, 0.53f, 1.0f)),
        ImColor(ImVec4(0.3f, 0.69f, 0.31f, 1.0f)),
        ImColor(ImVec4(0.55f, 0.76f, 0.29f, 1.0f)),
        ImColor(ImVec4(0.8f, 0.86f, 0.22f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.92f, 0.23f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.76f, 0.03f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.6f, 0.0f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.34f, 0.13f, 1.0f)),
        ImColor(ImVec4(0.47f, 0.33f, 0.28f, 1.0f)),
        ImColor(ImVec4(0.62f, 0.62f, 0.62f, 1.0f)),
        ImColor(ImVec4(0.38f, 0.49f, 0.55f, 1.0f))};

    /// OpenCV color definitions
    std::vector<cv::Scalar> colors = {
        cv::Scalar(54, 67, 244),
        cv::Scalar(99, 30, 233),
        cv::Scalar(176, 39, 156),
        cv::Scalar(183, 58, 103),
        cv::Scalar(181, 81, 63),
        cv::Scalar(243, 150, 33),
        cv::Scalar(244, 169, 3),
        cv::Scalar(212, 188, 0),
        cv::Scalar(136, 150, 0),
        cv::Scalar(80, 175, 76),
        cv::Scalar(74, 195, 139),
        cv::Scalar(57, 220, 205),
        cv::Scalar(59, 235, 255),
        cv::Scalar(7, 193, 255),
        cv::Scalar(0, 152, 255),
        cv::Scalar(34, 87, 255),
        cv::Scalar(72, 85, 121),
        cv::Scalar(158, 158, 158),
        cv::Scalar(139, 125, 96)};

public:
    KenningYolactGuiComponent(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Creates a /rosout RosData subscriber
        std::shared_ptr<RosRosoutSubscriberData> subscriber_rosout = std::make_shared<RosRosoutSubscriberData>(
            gui_node_ptr,
            "/rosout",
            [](const MsgRosoutSharedPtr msg) -> MsgRosoutSharedPtr { return msg; });
        gui_node_ptr->addRosData("rosout_subscriber", subscriber_rosout);

        // Adds a /rosout subscriber widget to the Node
        std::shared_ptr<RosoutWidget> rosout_widget =
            std::make_shared<RosoutWidget>(gui_node_ptr, "[Sub] /rosout logs", "rosout_subscriber", 10);
        gui_node_ptr->addWidget("rosout_widget", rosout_widget);

        // Creates a subscriber for instance segmentation results
        std::shared_ptr<RosSegmentationSubscriberData> subscriber_instance_segmentation = std::make_shared<RosSegmentationSubscriberData>(
            gui_node_ptr,
            "instance_segmentation_kenning",
            [](const kenning_computer_vision_msgs::msg::SegmentationMsg::SharedPtr msg) -> kenning_computer_vision_msgs::msg::SegmentationMsg::SharedPtr
            { return msg; });
        gui_node_ptr->addRosData("instance_segmentation_subscriber", subscriber_instance_segmentation);

        // Creates widget for displaying Kenning Instance Segmentation output
        std::shared_ptr<DetectionWidget> instance_segmentation_widget = std::make_shared<DetectionWidget>(
            gui_node_ptr,
            "[Sub] Instance Segmentation stream",
            "instance_segmentation_subscriber",
            [this](
                std::shared_ptr<GuiNode> gui_node_ptr,
                sensor_msgs::msg::Image &msg,
                std::vector<BoundingBox> &boxes,
                const std::string &filterclass,
                const float &threshold) -> void
            {
                std::shared_ptr<RosSegmentationSubscriberData> subscriber_instance_segmentation =
                    gui_node_ptr->getRosData("instance_segmentation_subscriber")->as<RosSegmentationSubscriberData>();
                kenning_computer_vision_msgs::msg::SegmentationMsg::SharedPtr instance_segmentation_msg = subscriber_instance_segmentation->getData();

                msg = prep_display(instance_segmentation_msg, boxes, filterclass, threshold);
            });
        gui_node_ptr->addWidget("instance_segmentation_widget", instance_segmentation_widget);

        // Creates a /camera_frame RosData subscriber
        std::shared_ptr<RosImageSubscriberData> subscriber_video = std::make_shared<RosImageSubscriberData>(
            gui_node_ptr,
            "camera_frame",
            [](const MsgImageSharedPtr msg) -> MsgImageSharedPtr { return msg; });
        gui_node_ptr->addRosData("video_subscriber", subscriber_video);

        // Create a widget to display the video stream
        std::shared_ptr<VideoWidget> video_widget = std::make_shared<VideoWidget>(
            gui_node_ptr,
            "[Sub] Video stream",
            "video_subscriber",
            [](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &msg) -> void
            {
                std::shared_ptr<RosImageSubscriberData> subscriber_video =
                    gui_node_ptr->getRosData("video_subscriber")->as<RosImageSubscriberData>();
                msg = *subscriber_video->getData().get();
            });
        gui_node_ptr->addWidget("video_widget", video_widget);

        // Create a widget to control parameters of the CameraNode
        std::shared_ptr<ControlWidget> control_widget = std::make_shared<ControlWidget>(
            gui_node_ptr, "[Control] Camera node", "camera_node");
        gui_node_ptr->addWidget("control_widget", control_widget);

        gui_node_ptr->prepare("Kenning Instance Segmentation GUI Node");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::KenningYolactGuiComponent)

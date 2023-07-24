/*
 * Copyright (c) 2022-2023 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cvnode_msgs/msg/segmentation_msg.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/utils/detection.hpp"
#include "gui_node/widget/widget_detection.hpp"
#include "gui_node/widget/widget_video.hpp"
#include "gui_node/widget/widget_rosout.hpp"

namespace gui_node
{

using RosYolactSubscriberData = RosSubscriberData<cvnode_msgs::msg::SegmentationMsg, cvnode_msgs::msg::SegmentationMsg::SharedPtr>;
using MsgImageSharedPtr = sensor_msgs::msg::Image::SharedPtr;
using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
using MsgRosoutSharedPtr = rcl_interfaces::msg::Log::SharedPtr;
using RosRosoutSubscriberData = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;

class YolactGuiComponent
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

    /**
     * Prepares the display for the YOLACT message.
     *
     * @param yolact_msg The segmentation message.
     * @param bounding_boxes The vector of bounding boxes to fill.
     * @param filterclass The class to filter masks by.
     * @param threshold The threshold to filter masks by.
     * @return sensor_msgs::msg::Image The image to display.
     */
    sensor_msgs::msg::Image prep_display(cvnode_msgs::msg::SegmentationMsg::SharedPtr yolact_msg,
                                         std::vector<BoundingBox> &bounding_boxes, const std::string &filterclass,
                                         const float &threshold)
    {
        float xmin, ymin, xmax, ymax;
        float score;
        int color_idx;
        std::string class_name;

        int height = yolact_msg->frame.height;
        int width = yolact_msg->frame.width;

        std::vector<uint8_t> masks = yolact_msg->masks;
        std::vector<float> boxes = yolact_msg->boxes;

        cv::Mat frame(height, width, CV_8UC3, yolact_msg->frame.data.data());

        for (int i = 0; i < yolact_msg->num_dets; i++)
        {
            class_name = yolact_msg->classes[i];
            score = yolact_msg->scores[i] * 100;
            color_idx = i % im_colors.size();

            // Add bounding box
            xmin = boxes[i * 4];
            ymin = boxes[i * 4 + 1];
            xmax = boxes[i * 4 + 2];
            ymax = boxes[i * 4 + 3];
            bounding_boxes.push_back(BoundingBox(xmin, ymin, xmax, ymax, class_name, score, im_colors[color_idx]));

            std::transform(class_name.begin(), class_name.end(), class_name.begin(), ::tolower);
            if (score >= threshold && (filterclass.empty() || class_name.find(filterclass) != std::string::npos))
            {
                // Apply mask
                cv::Mat mask(550, 550, CV_8UC1, masks.data() + i * 550 * 550);
                cv::resize(mask, mask, cv::Size(width, height));
                cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
                mask.setTo(colors[color_idx], mask);
                cv::addWeighted(mask, 0.4, frame, 1.0, 0, frame);
            }
        }
        return yolact_msg->frame;
    }

    /// ImGui color definitions
    std::vector<ImColor> im_colors{
        ImColor(ImVec4(0.96f, 0.26f, 0.21f, 1.0f)), ImColor(ImVec4(0.91f, 0.12f, 0.39f, 1.0f)),
        ImColor(ImVec4(0.61f, 0.15f, 0.69f, 1.0f)), ImColor(ImVec4(0.4f, 0.23f, 0.72f, 1.0f)),
        ImColor(ImVec4(0.25f, 0.32f, 0.71f, 1.0f)), ImColor(ImVec4(0.13f, 0.59f, 0.95f, 1.0f)),
        ImColor(ImVec4(0.01f, 0.66f, 0.96f, 1.0f)), ImColor(ImVec4(0.0f, 0.74f, 0.83f, 1.0f)),
        ImColor(ImVec4(0.0f, 0.59f, 0.53f, 1.0f)),  ImColor(ImVec4(0.3f, 0.69f, 0.31f, 1.0f)),
        ImColor(ImVec4(0.55f, 0.76f, 0.29f, 1.0f)), ImColor(ImVec4(0.8f, 0.86f, 0.22f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.92f, 0.23f, 1.0f)),  ImColor(ImVec4(1.0f, 0.76f, 0.03f, 1.0f)),
        ImColor(ImVec4(1.0f, 0.6f, 0.0f, 1.0f)),    ImColor(ImVec4(1.0f, 0.34f, 0.13f, 1.0f)),
        ImColor(ImVec4(0.47f, 0.33f, 0.28f, 1.0f)), ImColor(ImVec4(0.62f, 0.62f, 0.62f, 1.0f)),
        ImColor(ImVec4(0.38f, 0.49f, 0.55f, 1.0f))};

    /// OpenCV color definitions
    std::vector<cv::Scalar> colors = {
        cv::Scalar(54, 67, 244),  cv::Scalar(99, 30, 233),   cv::Scalar(176, 39, 156), cv::Scalar(183, 58, 103),
        cv::Scalar(181, 81, 63),  cv::Scalar(243, 150, 33),  cv::Scalar(244, 169, 3),  cv::Scalar(212, 188, 0),
        cv::Scalar(136, 150, 0),  cv::Scalar(80, 175, 76),   cv::Scalar(74, 195, 139), cv::Scalar(57, 220, 205),
        cv::Scalar(59, 235, 255), cv::Scalar(7, 193, 255),   cv::Scalar(0, 152, 255),  cv::Scalar(34, 87, 255),
        cv::Scalar(72, 85, 121),  cv::Scalar(158, 158, 158), cv::Scalar(139, 125, 96)};

public:
    YolactGuiComponent(const rclcpp::NodeOptions &options)
    {
        gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

        // Creates a /rosout RosData subscriber
        std::shared_ptr<RosRosoutSubscriberData> subscriber_rosout = std::make_shared<RosRosoutSubscriberData>(
            gui_node_ptr, "/rosout", [](const MsgRosoutSharedPtr msg) -> MsgRosoutSharedPtr { return msg; });
        gui_node_ptr->addRosData("rosout_subscriber", subscriber_rosout);

        // Adds a /rosout subscriber widget to the Node
        std::shared_ptr<RosoutWidget> rosout_widget =
            std::make_shared<RosoutWidget>(gui_node_ptr, "[Sub] /rosout logs", "rosout_subscriber", 10);
        gui_node_ptr->addWidget("rosout_widget", rosout_widget);

        // Creates a camera_frame_kenning RosData subscriber
        std::shared_ptr<RosYolactSubscriberData> subscriber_yolact = std::make_shared<RosYolactSubscriberData>(
            gui_node_ptr, "camera_frame_kenning",
            [](const cvnode_msgs::msg::SegmentationMsg::SharedPtr msg) -> cvnode_msgs::msg::SegmentationMsg::SharedPtr
            { return msg; });
        gui_node_ptr->addRosData("yolact_subscriber", subscriber_yolact);

        // Creates widget for displaying YOLACT output
        std::shared_ptr<DetectionWidget> yolact_widget = std::make_shared<DetectionWidget>(
            gui_node_ptr, "[Sub] Yolact stream", "yolact_subscriber",
            [this](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &msg, std::vector<BoundingBox> &boxes,
                   const std::string &filterclass, const float &threshold) -> void
            {
                std::shared_ptr<RosYolactSubscriberData> subscriber_yolact =
                    gui_node_ptr->getRosData("yolact_subscriber")->as<RosYolactSubscriberData>();
                cvnode_msgs::msg::SegmentationMsg::SharedPtr yolact_msg = subscriber_yolact->getData();

                msg = prep_display(yolact_msg, boxes, filterclass, threshold);
            });
        gui_node_ptr->addWidget("yolact_widget", yolact_widget);

        // Creates a /camera_frame RosData subscriber
        std::shared_ptr<RosImageSubscriberData> subscriber_video = std::make_shared<RosImageSubscriberData>(
            gui_node_ptr, "camera_frame", [](const MsgImageSharedPtr msg) -> MsgImageSharedPtr { return msg; });
        gui_node_ptr->addRosData("video_subscriber", subscriber_video);

        // Create a widget to display the video stream
        std::shared_ptr<VideoWidget> video_widget = std::make_shared<VideoWidget>(
            gui_node_ptr, "[Sub] Video stream", "video_subscriber",
            [](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &msg) -> void
            {
                std::shared_ptr<RosImageSubscriberData> subscriber_video =
                    gui_node_ptr->getRosData("video_subscriber")->as<RosImageSubscriberData>();
                msg = *subscriber_video->getData().get();
            });
        gui_node_ptr->addWidget("video_widget", video_widget);
        gui_node_ptr->prepare("Yolact GUI Node");
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gui_node_ptr->get_node_base_interface();
    }
};

} // namespace gui_node

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::YolactGuiComponent)

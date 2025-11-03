/*
 * Copyright (c) 2022-2025 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kenning_computer_vision_msgs/msg/box_msg.hpp>
#include <kenning_computer_vision_msgs/msg/frame_depth_estimation_msg.hpp>
#include <kenning_computer_vision_msgs/msg/frame_pose_estimation_msg.hpp>
#include <kenning_computer_vision_msgs/msg/frame_segmentation_msg.hpp>
#include <kenning_computer_vision_msgs/msg/pose_estimation_msg.hpp>
#include <kenning_computer_vision_msgs/msg/pose_msg.hpp>
#include <kenning_computer_vision_msgs/msg/segmentation_msg.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"
#include "gui_node/utils/detection.hpp"
#include "gui_node/widget/widget_control.hpp"
#include "gui_node/widget/widget_depth.hpp"
#include "gui_node/widget/widget_detection.hpp"
#include "gui_node/widget/widget_pose.hpp"
#include "gui_node/widget/widget_rosout.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

using RosSegmentationSubscriberData = RosSubscriberData<
    kenning_computer_vision_msgs::msg::FrameSegmentationMsg,
    kenning_computer_vision_msgs::msg::FrameSegmentationMsg::SharedPtr>;
using MsgImageSharedPtr = sensor_msgs::msg::Image::SharedPtr;
using RosImageSubscriberData = RosSubscriberData<sensor_msgs::msg::Image, sensor_msgs::msg::Image::SharedPtr>;
using MsgRosoutSharedPtr = rcl_interfaces::msg::Log::SharedPtr;
using RosRosoutSubscriberData = RosSubscriberData<rcl_interfaces::msg::Log, rcl_interfaces::msg::Log::SharedPtr>;
using MsgPoseEstimationPtr = kenning_computer_vision_msgs::msg::FramePoseEstimationMsg::SharedPtr;
using RosPoseEstimationSubscriberData = RosSubscriberData<
    kenning_computer_vision_msgs::msg::FramePoseEstimationMsg,
    kenning_computer_vision_msgs::msg::FramePoseEstimationMsg::SharedPtr>;
using DepthSharedPtr = kenning_computer_vision_msgs::msg::FrameDepthEstimationMsg::SharedPtr;
using RosDepthSubscriberData =
    RosSubscriberData<kenning_computer_vision_msgs::msg::FrameDepthEstimationMsg, DepthSharedPtr>;

class KenningMultiModelGuiComponent
{
private:
    std::shared_ptr<GuiNode> gui_node_ptr; ///< Pointer to the GUI node

    /**
     * Prepares the display for the Kenning's instance segmentation message.
     *
     * @param pose_estimation_msg The pose estimation message.
     * @param poses The vector of poses to draw.
     * @return sensor_msgs::msg::Image The image to display.
     */
    sensor_msgs::msg::Image prep_pose_display(MsgPoseEstimationPtr pose_estimation_msg, std::vector<Pose> &poses)
    {
        kenning_computer_vision_msgs::msg::VideoFrameMsg &frame = pose_estimation_msg->frame;

        kenning_computer_vision_msgs::msg::PoseEstimationMsg pose_estimation = pose_estimation_msg->estimation;

        const size_t pose_count = pose_estimation.poses.size();

        kenning_computer_vision_msgs::msg::SegmentationMsg segmentation = pose_estimation.segmentation;

        if ((pose_count != segmentation.classes.size()) || (pose_count != segmentation.boxes.size()) ||
            (pose_count != segmentation.scores.size()))

        {

            RCLCPP_ERROR(gui_node_ptr->get_logger(), "Pose count is inconsistent with segmentation count.");

            return frame.frame;
        }

        if ((segmentation.classes.size() != segmentation.boxes.size()) ||
            (segmentation.classes.size() != segmentation.scores.size()) ||
            (segmentation.boxes.size() != segmentation.scores.size()))
        {

            RCLCPP_ERROR(gui_node_ptr->get_logger(), "Segmentaion data are inconsistent.");

            return frame.frame;
        }

        for (size_t i = 0; i < pose_count; ++i)
        {
            if (segmentation.classes[i].length() == 0)
            {
                continue;
            }

            const int color_idx = i % im_colors.size();

            kenning_computer_vision_msgs::msg::PoseMsg pose = pose_estimation.poses[i];

            kenning_computer_vision_msgs::msg::BoxMsg box = segmentation.boxes[i];

            const std::string class_name = segmentation.classes[i];
            const float score = segmentation.scores[i] * 100.f;

            BoundingBox bbox(box.xmin, box.ymin, box.xmax, box.ymax, class_name, score, im_colors[color_idx]);

            std::vector<Point> keypoints;

            for (const auto &keypoint : pose.keypoints)
            {
                keypoints.push_back(Point(keypoint.x, keypoint.y, keypoint.id));
            }

            poses.push_back(Pose(keypoints, bbox));
        }

        return frame.frame;
    }

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
        kenning_computer_vision_msgs::msg::FrameSegmentationMsg::SharedPtr instance_segmentation_msg,
        std::vector<BoundingBox> &bounding_boxes,
        const std::string &filterclass,
        const float &threshold)
    {

        kenning_computer_vision_msgs::msg::VideoFrameMsg &frame = instance_segmentation_msg->frame;

        kenning_computer_vision_msgs::msg::BoxMsg box;

        const int height = frame.frame.height;
        const int width = frame.frame.width;

        cv::Mat mat_frame(height, width, CV_8UC3, frame.frame.data.data());

        kenning_computer_vision_msgs::msg::SegmentationMsg &segmentation = instance_segmentation_msg->segmentation;

        for (size_t i = 0; i < segmentation.boxes.size(); i++)
        {
            std::string class_name = segmentation.classes[i];
            const float score = segmentation.scores[i] * 100;
            const int color_idx = i % im_colors.size();

            box = segmentation.boxes[i];

            bounding_boxes.push_back(
                BoundingBox(box.xmin, box.ymin, box.xmax, box.ymax, class_name, score, im_colors[color_idx]));

            std::transform(class_name.begin(), class_name.end(), class_name.begin(), ::tolower);
            if (score >= threshold && (filterclass.empty() || class_name.find(filterclass) != std::string::npos))
            {
                // Apply mask
                cv::Mat mask(
                    segmentation.masks[i].dimension[0],
                    segmentation.masks[i].dimension[1],
                    CV_8UC1,
                    segmentation.masks[i].data.data());
                cv::resize(mask, mask, cv::Size(width, height));
                cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
                mask.setTo(colors[color_idx], mask);
                cv::addWeighted(mask, 0.4, mat_frame, 1.0, 0, mat_frame);
            }
        }
        return frame.frame;
    }

    /// ImGui color definitions
    const std::vector<ImColor> im_colors{
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
    const std::vector<cv::Scalar> colors = {
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
    KenningMultiModelGuiComponent(const rclcpp::NodeOptions &options)
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

        // Creates a subscriber for depth estimation results
        std::shared_ptr<RosDepthSubscriberData> subscriber_depth = std::make_shared<RosDepthSubscriberData>(
            gui_node_ptr,
            "depth_estimation_kenning",
            [](const DepthSharedPtr msg) -> DepthSharedPtr { return msg; });
        gui_node_ptr->addRosData("depth_subscriber", subscriber_depth);
        // Create a widget to display the depth information
        std::shared_ptr<DepthWidget> depth_widget = std::make_shared<DepthWidget>(
            gui_node_ptr,
            "[Sub] Depth stream",
            "depth_subscriber",
            [this](std::shared_ptr<GuiNode> gui_node_ptr, DepthWidget::DepthData &depth) -> void
            {
                std::shared_ptr<RosDepthSubscriberData> subscriber_depth =
                    gui_node_ptr->getRosData("depth_subscriber")->as<RosDepthSubscriberData>();
                auto &depth_msg = subscriber_depth->getData().get()->depth;

                depth.values = depth_msg.data;
                depth.rows = depth_msg.rows;
                depth.cols = depth_msg.cols;
            });
        gui_node_ptr->addWidget("depth_widget", depth_widget);

        // Creates a subscriber for instance segmentation results
        std::shared_ptr<RosSegmentationSubscriberData> subscriber_instance_segmentation =
            std::make_shared<RosSegmentationSubscriberData>(
                gui_node_ptr,
                "instance_segmentation_kenning",
                [](const kenning_computer_vision_msgs::msg::FrameSegmentationMsg::SharedPtr msg)
                    -> kenning_computer_vision_msgs::msg::FrameSegmentationMsg::SharedPtr { return msg; });
        gui_node_ptr->addRosData("instance_segmentation_subscriber", subscriber_instance_segmentation);

        // Creates a subscriber for pose estimation results
        std::shared_ptr<RosPoseEstimationSubscriberData> subscriber_pose_estimation =
            std::make_shared<RosPoseEstimationSubscriberData>(
                gui_node_ptr,
                "pose_estimation_kenning",
                [](const MsgPoseEstimationPtr msg) -> MsgPoseEstimationPtr { return msg; });
        gui_node_ptr->addRosData("pose_estimation_subscriber", subscriber_pose_estimation);

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
                kenning_computer_vision_msgs::msg::FrameSegmentationMsg::SharedPtr instance_segmentation_msg =
                    subscriber_instance_segmentation->getData();

                msg = prep_display(instance_segmentation_msg, boxes, filterclass, threshold);
            });
        gui_node_ptr->addWidget("instance_segmentation_widget", instance_segmentation_widget);

        // Creates widget for displaying Kenning Pose Estimation output
        std::shared_ptr<PoseWidget> pose_estimation_widget = std::make_shared<PoseWidget>(
            gui_node_ptr,
            "[Sub] Pose estimation",
            "pose_estimation_subscriber",
            [this](std::shared_ptr<GuiNode> gui_node_ptr, sensor_msgs::msg::Image &msg, std::vector<Pose> &poses)
                -> void
            {
                std::shared_ptr<RosPoseEstimationSubscriberData> subscriber_pose_estimation =
                    gui_node_ptr->getRosData("pose_estimation_subscriber")->as<RosPoseEstimationSubscriberData>();

                MsgPoseEstimationPtr pose_estimation_msg = subscriber_pose_estimation->getData();

                // extract data
                msg = prep_pose_display(pose_estimation_msg, poses);
            });
        gui_node_ptr->addWidget("pose_estimation_widget", pose_estimation_widget);

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
        std::shared_ptr<ControlWidget> control_widget =
            std::make_shared<ControlWidget>(gui_node_ptr, "[Control] Camera node", "camera_node");
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

RCLCPP_COMPONENTS_REGISTER_NODE(gui_node::KenningMultiModelGuiComponent)

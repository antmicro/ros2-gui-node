#pragma once

#include "gui_node/utils/detection.hpp"
#include "gui_node/widget/widget_video.hpp"

namespace gui_node
{

using BBoxExtractor = std::function<void(std::shared_ptr<GuiNode>, sensor_msgs::msg::Image &,
                                         std::vector<BoundingBox> &, const std::string &, const float &)>;
/**
 * Class for object detection widgets.
 */
class DetectionWidget : public BaseVideoWidget
{
private:
    /**
     * Extracts bounding boxes and image form the message.
     *
     * @param gui_node The shared pointer to the GUI node.
     * @param image The image message.
     */
    void frame_extractor(std::shared_ptr<GuiNode> gui_node, sensor_msgs::msg::Image &image);

    /**
     * Draws bounding boxes on the ImGui window.
     */
    void imgui_callback();

    const float base_cornerroundingfactor = 10.0f;      ///< Base corner rounding factor.
    const float base_perimeterthickness = 8.0f;         ///< Base perimeter thickness.
    float threshold = 0;                                ///< Threshold for the bounding boxes to be drawn.
    ImColor hiddenobjectcolor = ImColor(128, 128, 128); ///< Color for the hidden objects.
    std::string filterclass;                            ///< Filter class for the bounding boxes to be drawn.
    std::vector<BoundingBox> bounding_boxes;            ///< Vector of bounding boxes.

    BBoxExtractor data_extractor; ///< Function to extract bounding boxes and image form the message.
public:
    /**
     * Constructor.
     *
     * @param gui_node The shared pointer to the GUI node.
     * @param window_name The name for the ImGui window.
     * @param ros_data_name Name of the ROSData object that widget is associated with.
     * @param data_extractor Function to extract bounding boxes and image form the message.
     */
    DetectionWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name,
                    BBoxExtractor data_extractor)
        : BaseVideoWidget(
              gui_node, window_name, ros_data_name,
              std::bind(&DetectionWidget::frame_extractor, this, std::placeholders::_1, std::placeholders::_2),
              std::bind(&DetectionWidget::imgui_callback, this))
    {
        this->data_extractor = data_extractor;
        bounding_boxes = std::vector<BoundingBox>();
    }
};

} // namespace gui_node

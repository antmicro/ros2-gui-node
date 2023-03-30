#pragma once

#include <string>

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

/**
 * Base class for string widgets.
 */
class BaseStringWidget : public Widget
{
private:
    std::string data = "";

protected:
    /**
     * Get the data from the ROS data object.
     *
     * @return std::string The data.
     */
    virtual std::string getData() = 0;

public:
    BaseStringWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name,
                     const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draw the widget.
     */
    void draw() override;
};

/**
 * Widget for displaying string data from a subscriber.
 */
class StringSubscriberWidget : public BaseStringWidget
{
private:
    /**
     * Get the data from the ROS data subscriber object.
     *
     * @return std::string The data.
     */
    std::string getData();

public:
    StringSubscriberWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name,
                           const std::string &ros_data_name)
        : BaseStringWidget(gui_node, window_name, ros_data_name)
    {
    }
};

/**
 * Widget for displaying string data from a publisher.
 */
class StringPublisherWidget : public Widget
{
private:
    /**
     * Get the data from the ROS data publisher object.
     *
     * @return std::string The data.
     */
    std::string getData();

public:
    StringPublisherWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name,
                          const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }
};

} // namespace gui_node

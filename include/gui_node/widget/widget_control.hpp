#pragma once

#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget.hpp"

#include <imgui.h>
#include <map>
#include <memory>
#include <mutex>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

namespace gui_node
{

using namespace rcl_interfaces::srv;
using namespace rcl_interfaces::msg;

/**
 * Structure representing a parameter of the ROS2 node.
 */
struct NodeParameter
{
    std::string name;                          ///< Name of the parameter
    std::string type;                          ///< Type of the parameter
    ParameterValue::SharedPtr value;           ///< Value of the parameter
    ParameterDescriptor::SharedPtr descriptor; ///< Descriptor of the parameter

    /**
     * Compares two parameters.
     *
     * @param sort_specs Sort specifications.
     * @param a First parameter.
     * @param b Second parameter.
     *
     * @return True if the first parameter is less than the second one, false otherwise.
     */
    static bool compare(ImGuiTableSortSpecs *sort_specs, const NodeParameter &a, const NodeParameter &b);
};

/**
 * Widget for controlling ROS2 node parameters.
 */
class ControlWidget : public Widget
{
private:
    /**
     * Gets the list of parameters of the ROS2 node.
     */
    void getParameters();

    /**
     * Gets descriptors for defined parameters of the ROS2 node.
     */
    void getParametersDescriptors();

    /**
     * Gets values for defined parameters of the ROS2 node.
     */
    void getParametersValue();

    /**
     * Draws name of a parameter.
     *
     * @param parameter Parameter to draw name of.
     */
    void drawName(const NodeParameter &parameter);

    /**
     * Draws type of a parameter.
     *
     * @param parameter Parameter to draw type of.
     */
    void drawType(NodeParameter &parameter);

    /**
     * Draws value of a parameter.
     *
     * @param parameter Parameter to draw the value of.
     */
    void drawValue(const NodeParameter &parameter);

    /**
     * Draws numeric value of a parameter.
     *
     * @param from_val Lower bound of the parameter, can be empty.
     * @param to_val Upper bound of the parameter, can be empty.
     * @param draw_function Function to draw the value.
     */
    void drawNumericValue(const std::string &from_val, const std::string &to_val, std::function<void()> draw_function);

    /**
     * Draws array value of a parameter.
     *
     * @param parameter Parameter to draw the value of.
     * @param array Array to draw the value of.
     * @param zero_value Default value to append to the array.
     * @param draw_function Function to draw the value. Takes width of drawing area and index of the array element as
     * arguments.
     *
     * @tparam T Type of the array elements.
     */
    template <typename T>
    void drawArrayValue(
        const NodeParameter &parameter,
        std::vector<T> &array,
        T zero_value,
        std::function<void(float, size_t)> draw_function);

    /**
     * Updates the value of a parameters.
     */
    void updateParameters();

    /**
     * Sorts table using specification if changed.
     */
    void sortTable();

    /**
     * Removes trailing zeros from a string.
     *
     * @param str String to remove trailing zeros from.
     *
     * @return String without trailing zeros.
     */
    std::string trimZeros(const std::string &str);

    /// Client to list parameters of the ROS2 node
    rclcpp::Client<ListParameters>::SharedPtr list_parameters_client_;
    /// Client to get parameters of the ROS2 node
    rclcpp::Client<GetParameters>::SharedPtr get_parameters_client_;
    /// Client to describe parameters of the ROS2 node
    rclcpp::Client<DescribeParameters>::SharedPtr describe_parameters_client_;
    // Client to set parameters of the ROS2 node
    rclcpp::Client<SetParameters>::SharedPtr set_parameters_client_;
    /// Map of parameter types
    std::map<uint8_t, std::string> parameter_types{
        {ParameterType::PARAMETER_NOT_SET, "Parameter not set"},
        {ParameterType::PARAMETER_BOOL, "Boolean"},
        {ParameterType::PARAMETER_INTEGER, "Integer"},
        {ParameterType::PARAMETER_DOUBLE, "Double"},
        {ParameterType::PARAMETER_STRING, "String"},
        {ParameterType::PARAMETER_BYTE_ARRAY, "Byte array"},
        {ParameterType::PARAMETER_BOOL_ARRAY, "Boolean array"},
        {ParameterType::PARAMETER_INTEGER_ARRAY, "Integer array"},
        {ParameterType::PARAMETER_DOUBLE_ARRAY, "Double array"},
        {ParameterType::PARAMETER_STRING_ARRAY, "String array"}};
    /// Flags for a table with parameters
    ImGuiTableFlags table_flags = ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_RowBg | ImGuiTableFlags_Sortable | ImGuiTableFlags_SortMulti |
                                  ImGuiTableFlags_SortTristate | ImGuiTableFlags_ScrollY;

    bool first_draw = true;                ///< Flag indicating if the widget is drawn for the first time
    std::mutex parameters_mutex;           ///< Mutex for accessing the parameters
    std::string curr_param_desc;           ///< Description of the currently selected parameter
    std::string filter_name;               ///< Name of the filter for parameters
    std::vector<NodeParameter> parameters; ///< Map of parameters of the ROS2 node

public:
    /**
     * Constructor.
     *
     * @param gui_node Pointer to the GUI node the widget is associated with.
     * @param window_name Name of the ImGui window.
     * @param ros_node_name Name of the ROS2 node to control parameters of.
     */
    ControlWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_node_name);

    /**
     * Draws the widget using ImGui backend.
     */
    void draw() override;
};

} // namespace gui_node

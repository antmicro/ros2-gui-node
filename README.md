# ROS2 GuiNode

Copyright (c) 2022-2024 [Antmicro](https://www.antmicro.com)

`GuiNode` is a library for visualizing data from ROS2 topics and services.
It provides tools for manipulating Widgets and data objects, which can be used for data visualization.
The graphical user interface is implemented using the Vulkan API, GLFW3, and Dear ImGui libraries.

## GUI node examples

* [Kenning instance segmentation](examples/kenning-instance-segmentation/) - runs GUI node visualizing instance segmentation from YOLACT optimized using Kenning framework.
* [Simple GUI node demonstration, built with the project](src/samples/sample_gui_node.cpp)

## Building the GuiNode

Project dependencies:

* [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
* `OpenCV`
* `Vulkan`
* `GLFW3`

The `GuiNode` uses the `colcon` build system to build the project.
First, you need to source the `ROS2` environment, and then you can use the `colcon` command to build the project.
You should replace `<path_to_ros2_env>` with the path to the ROS2 environment setup script (e.g. `/opt/ros/setup.bash`).

```bash
source <path_to_ros2_env>
cd <path_to_gui_node_repo>
colcon build
```

You can find the artifacts from the building stage in the `install/` directory. They contain setup scripts for the `GuiNode` environment.

```bash
source install/local_setup.bash
```

Once the `GuiNode` environment is sourced, you can run the `GuiNode` sample using the `ros2 launch` command:

```bash
ros2 launch gui_node sample_launch.py
```

For more information about the samples, go to the `src/samples` directory.

## Widgets and RosData objects

`GuiNode` offers a number of pre-implemented Widgets which can be used to visualize the data.
Widgets are responsible for creating the GUI elements and displaying the data.

The following Widgets are already implemented in the `GuiNode`:

* `VideoWidget` - displays an image from the `sensor_msgs::msg::Image` message type.
* `StringWidget` - displays the `std::string` data in a text box.
* `RosoutWidget` - displays messages from the `/rosout` topic (the ROS2 logging topic) in table view.
* `DetectionWidget` - displays an image from the `sensor_msgs::msg::Image` message type and draws bounding boxes.

Widgets are using `RosData` objects to get fresh data from topics or services.
You should create the corresponding `RosData` object add it to the `GuiNode` before calling the `gui_node->prepare()` method.

The `RosData` objects are responsible for handling data from ROS2 topics or services.

Below is an example of how to add `RosoutWidget` to the `GuiNode` instance:

```cpp
#include "gui_node/gui_node.hpp"
#include "gui_node/widget/widget_rosout.hpp"
#include "gui_node/ros_data/ros_subscriber_data.hpp"

...

gui_node_ptr = std::make_shared<GuiNode>(options, "gui_node");

// Creates a /rosout RosData subscriber
std::shared_ptr<RosRosoutSubscriberData> subscriber_rosout = std::make_shared<RosRosoutSubscriberData>(
    gui_node_ptr, "/rosout", [](const MsgRosoutSharedPtr msg) -> MsgRosoutSharedPtr { return msg; });
gui_node_ptr->addRosData("rosout_subscriber", subscriber_rosout);

// Adds a /rosout subscriber Widget to the Node
std::shared_ptr<RosoutWidget> rosout_widget =
    std::make_shared<RosoutWidget>(gui_node_ptr, "[Sub] /rosout logs", "rosout_subscriber", 10);
gui_node_ptr->addWidget("rosout_widget", rosout_widget);

// Prepare the GuiNode and start drawing the GUI
gui_node_ptr->prepare("Window name");

...
```

`GuiNode` offers the following set of implemented `RosData` objects:

* `RosPublisherData` - publishes data to a ROS2 topic and saves the last published data.
* `RosSubscriberData` - subscribes to a ROS2 topic and provides the last received data.
* `RosServiceServerData` - provides a service server that saves data from the last processed request.
* `RosServiceClientData` - provides a service client that can be used to send a request to the server and save data from the response.

For more information about `RosData` objects, see the `include/gui_node/ros_data` directory, and for more information about Widgets, see the `include/gui_node/widget` directory.

## Implementing a Widget

You can create Widgets by inheriting the `Widget` class and implementing the `draw` method.
`GuiNode` calls a Widget's `draw` method every frame and the Widget is responsible for displaying data and handling user input.

### Creating a new Widget

In this section, you will see how to create a new Widget that displays a counter value and provides a button for incrementing it.

The obligatory parameters for the `Widget` constructor are:
* `gui_node` - a `GuiNode` shared pointer that will be used for logging and accessing `RosData` objects;
* `window_name` - the name of the Widget's window;
* `ros_data_name` - a unique name of the `RosData` object that will be used to reference the object with data in the Widget objects during render.

A sample declaration of the `CounterWidget` class (the example can be found in `include/gui_node/widget/widget_counter.hpp` and `src/gui_node/widget/widget_counter.cpp` files):

```cpp
#include <memory>
#include <string>

#include <std_srvs/srv/trigger.hpp>

#include "gui_node/gui_node.hpp"
#include "gui_node/ros_data/ros_client_data.hpp"
#include "gui_node/widget/widget.hpp"

namespace gui_node
{

class CounterWidget : public Widget
{
private:
    int counter = 0; ///< Counter value

public:
    CounterWidget(std::shared_ptr<GuiNode> gui_node, const std::string &window_name, const std::string &ros_data_name)
        : Widget(gui_node, window_name, ros_data_name)
    {
    }

    /**
     * Draw the Widget
     */
    void draw() override {
        // Get the data
        using RosCounterClientData = RosServiceClientData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response>;
        std::shared_ptr<RosCounterClientData> ros_data =
            this->gui_node->getRosData(ros_data_name)->as<RosCounterClientData>();
        if (ros_data->hasDataChanged())
        {
            std_srvs::srv::Trigger::Response response = ros_data->getData();
            if (response.success && response.message == "triggered")
            {
                counter++;
            }
        }

        // Draw the Widget
        ImGui::Begin(window_name.c_str());
        ImGui::SetWindowSize(ImVec2(200, 100), ImGuiCond_FirstUseEver);
        ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize("Trigger").x) / 2);

        // If service is not available, disable the button
        if (ros_data->isServiceAvailable())
        {
            if (ImGui::Button("Trigger"))
            {
                // Make the request
                std_srvs::srv::Trigger::Request::SharedPtr request = std::make_shared<std_srvs::srv::Trigger::Request>();
                ros_data->sendRequest(request);
            }
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.0f, 0.7f, 0.7f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.0f, 0.8f, 0.8f));
            ImGui::Button("Trigger");
            ImGui::PopStyleColor(3);
        }
        std::string counter_str = "Counter: " + std::to_string(counter);
        ImGui::SetCursorPosX((ImGui::GetWindowWidth() - ImGui::CalcTextSize(counter_str.c_str()).x) / 2);
        ImGui::Text("%s", counter_str.c_str());
        ImGui::End();
    }
};

} // namespace gui_node
```

The overridden `draw` method is responsible for drawing the Widget and called by the `GuiNode` every time the drawing loop is executed.
Usually, the `draw` method uses the `getRosData` method of the `GuiNode` to get the `RosData` object, and later visualizes data from it.

The `GuiEngine` class expects Widgets to be drew using the `Dear ImGui` library.

The `draw` method obtains the `RosCounterClientData` object from the `GuiNode`, and verifies if the data has changed since the last invocation of the `draw` method.
If new data is available, the response is verified to be successful with counter value being incremented.

When the service is not available, the button is disabled.
This happens to prevent the user from sending requests to the service server when it is not available, which would result in an error.

### Adding the Widget to the sample node

The `src/samples/` directory contains examples on how to implement the ROS2 component node and integrate it with the `GuiNode`.
It can be used as a reference for creating new nodes.

In this section, the `CounterWidget` Widget is added to the `src/samples/sample_gui_node.cpp` file by following an example from the [Widgets and RosData objects](#widgets-and-rosdata-objects) section:
```cpp
#include <std_srvs/srv/trigger.hpp>
#include "gui_node/widget/widget_counter.hpp"

...

using RosCounterClientData = RosServiceClientData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response>;

...

SampleGuiComponent(const rclcpp::NodeOptions &options)
{
    ...

    // Create a /counter RosData service client
    std::shared_ptr<RosCounterClientData> client_counter = std::make_shared<RosCounterClientData>(
        gui_node_ptr, "/counter",
        [](std_srvs::srv::Trigger::Response::SharedPtr response) -> std_srvs::srv::Trigger::Response::SharedPtr
        { return response; });
    gui_node_ptr->addRosData("counter_service", client_counter);

    // Create a counter Widget
    std::shared_ptr<CounterWidget> counter_widget =
        std::make_shared<CounterWidget>(gui_node_ptr, "[Client] Counter", "counter_service");
    gui_node_ptr->addWidget("counter_widget", counter_widget);

    gui_node_ptr->prepare("Sample GUI widgets");
}

...
```

This code creates a `RosData` service client object responsible for sending requests to the `/counter` service server and receiving responses.
The `CounterWidget` object is created and added to the `GuiNode` using the `addWidget` method.

Without the `/counter` server, the increment button will be disabled.
The corresponding service server is created in the `src/samples/sample_publisher_node.cpp` file, which is responsible for publishing data for the sample:

```cpp
#include <std_srvs/srv/trigger.hpp>
#include "gui_node/ros_data/ros_server_data.hpp"

...

using RosCounterServerData = RosServiceServerData<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Response::SharedPtr>;

...

SampleGuiComponent(const rclcpp::NodeOptions &options)
{
    ...

    // Create the /counter RosData service server
    std::shared_ptr<RosCounterServerData> ros_server_data_ptr = std::make_shared<RosCounterServerData>(
    gui_node_ptr, "/counter",
    [](std_srvs::srv::Trigger::Request::SharedPtr request,
       std_srvs::srv::Trigger::Response::SharedPtr response) -> std_srvs::srv::Trigger::Response::SharedPtr
    {
        response->success = true;
        response->message = "triggered";
        return response;
    });
    gui_node_ptr->addRosData("counter_server", ros_server_data_ptr);
}

...
```

This code creates a `RosData` service server object which is responsible for receiving requests from the `/counter` service client and sending back responses.

Now, you can build and run the sample GUI node with the Widget working as expected:

```bash
source <path_to_ros2_env>
cd <path_to_gui_node_repo>
colcon build
source install/local_setup.bash
ros2 launch gui_node sample_launch.py
```

## Code formatting

Formatting dependencies:

* `ament_uncrustify` (ROS2 package)
* `clang-format`
* `clang-tidy`

The `GuiNode` uses the `ament_clang_format` and `ament_clang_tidy` packages to verify code formatting and lint the code.
To run the lint checks, build the `GuiNode` and then use the `colcon test` command to run the lint checks.

```bash
source <path_to_ros2_env>
cd <path_to_gui_node_repo>
colcon build
colcon test
colcon test-result --all
```

In case of any lint errors, the `colcon test-result` command will print a list of files that contain errors.
The `ament_clang_format` and `ament_clang_tidy` packages can be used to automatically fix formatting issues:

```bash
cd <path_to_gui_node_repo>
ament_clang_format --config .clang-format --reformat <path_to_files_or_directories>
```

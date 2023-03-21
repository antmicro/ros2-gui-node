cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(gui_node VERSION 0.0.1)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Vulkan REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
    third-party/imgui
    third-party/imgui/backends
    Vulkan::Headers
)

set(IMGUI_SOURCES
    third-party/imgui/backends/imgui_impl_glfw.cpp
    third-party/imgui/backends/imgui_impl_vulkan.cpp
    third-party/imgui/imgui_draw.cpp
    third-party/imgui/imgui_tables.cpp
    third-party/imgui/imgui_widgets.cpp
    third-party/imgui/imgui.cpp
)

add_compile_definitions(GLFW_INCLUDE_VULKAN)

#######################
# GuiNode library
#######################
add_library(gui_node SHARED
    src/gui_node.cpp
    src/gui_engine.cpp
    ${IMGUI_SOURCES}
)

target_link_libraries(gui_node
    glfw
    Vulkan::Vulkan
)

ament_target_dependencies(gui_node rclcpp)

#######################
# SampleGuiComponent
#######################
add_library(sample_gui_node SHARED
    src/samples/sample_gui_node.cpp
    src/widget/widget_rosout.cpp
    src/widget/widget_video.cpp
    src/widget/widget_string.cpp
)

target_link_libraries(sample_gui_node
    gui_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(sample_gui_node
    rcl_interfaces
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

rclcpp_components_register_nodes(sample_gui_node "gui_node::SampleGuiComponent")

#######################
# SamplePublisherComponent
#######################
add_library(sample_publish_node SHARED
    src/samples/sample_publisher_node.cpp
)

target_link_libraries(sample_publish_node
    gui_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(sample_publish_node
    rclcpp
    rclcpp_components
    std_msgs
    sensor_msgs
)

rclcpp_components_register_nodes(sample_publish_node "gui_node::SamplePublisherComponent")

#######################
# Install
#######################
install(TARGETS
    gui_node
    sample_gui_node
    sample_publish_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install (FILES launch/sample_launch.py
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_clang_tidy REQUIRED)

    ament_clang_format(CONFIG_FILE ".clang-format"
        "include"
        "src"
    )

    ament_clang_tidy(CONFIG_FILE ".clang-format"
        ${CMAKE_BINARY_DIR}/compile_commands.json
        "include"
        "src"
    )
endif()

ament_package()

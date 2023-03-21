cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(gui_node VERSION 0.0.1)

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

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

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
# Time publisher node
#######################
add_library(gui_node_sample_publisher SHARED src/samples/string_publisher_node.cpp)

target_link_libraries(gui_node_sample_publisher gui_node)

ament_target_dependencies(gui_node_sample_publisher
    rclcpp
    rclcpp_components
    std_msgs
)

rclcpp_components_register_nodes(gui_node_sample_publisher "gui_node::StringPublisher")

#######################
# Time subscriber node
#######################
add_library(gui_node_sample_subscriber SHARED src/samples/string_subscriber_node.cpp)

target_link_libraries(gui_node_sample_subscriber gui_node)

ament_target_dependencies(gui_node_sample_subscriber
    rclcpp
    rclcpp_components
    std_msgs
)

rclcpp_components_register_nodes(gui_node_sample_subscriber "gui_node::StringSubscriber")

#######################
# Rosout node
#######################
add_library(gui_node_sample_rosout SHARED
    src/samples/rosout_subscriber_node.cpp
    src/widget/widget_rosout.cpp
)

target_link_libraries(gui_node_sample_rosout gui_node)

ament_target_dependencies(gui_node_sample_rosout
    rclcpp
    rclcpp_components
    rcl_interfaces
)

rclcpp_components_register_nodes(gui_node_sample_rosout "gui_node::RosoutSubscriber")

#######################
# Video publisher node
#######################
add_library(gui_node_sample_video_publish SHARED
    src/samples/video_publisher_node.cpp
    src/widget/widget_video.cpp
)

target_link_libraries(gui_node_sample_video_publish
    gui_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(gui_node_sample_video_publish
    rclcpp
    rclcpp_components
    sensor_msgs
)

rclcpp_components_register_nodes(gui_node_sample_video_publish "gui_node::VideoPublisher")

#######################
# Video subscriber node
#######################
add_library(gui_node_sample_video_subscribe SHARED
    src/samples/video_subscriber_node.cpp
    src/widget/widget_video.cpp
)

target_link_libraries(gui_node_sample_video_subscribe
    gui_node
    ${OpenCV_LIBS}
)

ament_target_dependencies(gui_node_sample_video_subscribe
    rclcpp
    rclcpp_components
    sensor_msgs
)

rclcpp_components_register_nodes(gui_node_sample_video_subscribe "gui_node::VideoSubscriber")

#######################
# Install
#######################
install(TARGETS
    gui_node
    gui_node_sample_publisher
    gui_node_sample_subscriber
    gui_node_sample_rosout
    gui_node_sample_video_publish
    gui_node_sample_video_subscribe
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install (FILES launch/sample_launch.py
    DESTINATION share/${PROJECT_NAME}
)

add_custom_target(format
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-tidy -format-style=file -p ${CMAKE_BINARY_DIR} -fix"
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-format --style=file -i"
    VERBATIM
)

ament_package()

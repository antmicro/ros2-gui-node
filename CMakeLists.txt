cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(gui_node VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Vulkan REQUIRED)

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

set(node_plugins "")

add_compile_definitions(GLFW_INCLUDE_VULKAN)

#######################
# GUI Node
#######################
add_library(gui_node SHARED
    src/gui_node.cpp
)
ament_target_dependencies(gui_node
    rclcpp
    rclcpp_components
    std_msgs
)
rclcpp_components_register_nodes(gui_node "gui_node::GuiNode")
set(node_plugins "${gui_node}gui_node::GuiNode;$<TARGET_FILE:gui_node>\n")

#######################
# Publish node
#######################
add_library(gui_node_sample_publisher SHARED
    src/samples/publisher_node.cpp
    src/gui_node.cpp
)

ament_target_dependencies(gui_node_sample_publisher
    rclcpp
    rclcpp_components
    std_msgs
)
rclcpp_components_register_nodes(gui_node_sample_publisher "gui_node::TimePublisher")
set(node_plugins "${gui_node_sample_publisher}gui_node::TimePublisher;$<TARGET_FILE:publisher_node>\n")

#######################
# Install
#######################
install(TARGETS
    gui_node
    gui_node_sample_publisher
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

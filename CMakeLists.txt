# Copyright 2022-2024 Antmicro <www.antmicro.com>
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(gui_node VERSION 0.0.1)

option(BUILD_KENNING_YOLACT_DEMO "Builds demo application running visualization of YOLACT instance segmentation with Kenning" OFF)

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
find_package(std_srvs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Vulkan REQUIRED)
find_package(OpenCV REQUIRED)

set(WIDGET_SOURCES
    src/widget/widget_control.cpp
    src/widget/widget_counter.cpp
    src/widget/widget_detection.cpp
    src/widget/widget_pose.cpp
    src/widget/widget_depth.cpp
    src/widget/widget_rosout.cpp
    src/widget/widget_string.cpp
    src/widget/widget_video.cpp
)

set(IMGUI_SOURCES
    third-party/imgui/backends/imgui_impl_glfw.cpp
    third-party/imgui/backends/imgui_impl_vulkan.cpp
    third-party/imgui/misc/cpp/imgui_stdlib.cpp
    third-party/imgui/imgui_draw.cpp
    third-party/imgui/imgui_tables.cpp
    third-party/imgui/imgui_widgets.cpp
    third-party/imgui/imgui.cpp
)

add_compile_definitions(GLFW_INCLUDE_VULKAN)

#######################
# GuiNode library
#######################
add_library(${PROJECT_NAME} SHARED
    src/gui_node.cpp
    src/gui_engine.cpp
    ${IMGUI_SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    glfw
    ${Vulkan_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${Vulkan_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/imgui>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/imgui/backends>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/imgui/misc/cpp>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY
    include/
    third-party/imgui/
    third-party/imgui/backends/
    third-party/imgui/misc/cpp/
    DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

install(TARGETS
    ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

export(TARGETS
    ${PROJECT_NAME}
    FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

#######################
# Core widget library
#######################
add_library(${PROJECT_NAME}_core_widgets SHARED
    ${WIDGET_SOURCES}
)

target_include_directories(${PROJECT_NAME}_core_widgets PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_core_widgets
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_core_widgets
    rcl_interfaces
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
)

ament_export_targets(${PROJECT_NAME}_core_widgetsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(rcl_interfaces
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
)

install(TARGETS
    ${PROJECT_NAME}_core_widgets
    EXPORT ${PROJECT_NAME}_core_widgetsTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

export(TARGETS
    ${PROJECT_NAME}_core_widgets
    FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}_core_widgetsTargets.cmake
)

#######################
# SampleGuiComponent
#######################
add_library(sample_gui_node SHARED
    src/samples/sample_gui_node.cpp
)

target_link_libraries(sample_gui_node
    ${PROJECT_NAME}
    ${PROJECT_NAME}_core_widgets
)

ament_target_dependencies(sample_gui_node
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    std_srvs
)

rclcpp_components_register_nodes(sample_gui_node "gui_node::SampleGuiComponent")

if (BUILD_KENNING_YOLACT_DEMO)
    # package with necessary messages to communicate with Kenning
    find_package(kenning_computer_vision_msgs REQUIRED)

    #######################
    # KenningYolactGuiComponent
    #######################
    add_library(kenning_yolact_gui_node SHARED
        examples/kenning-instance-segmentation/kenning_yolact_gui_node.cpp
    )

    target_link_libraries(kenning_yolact_gui_node
        ${PROJECT_NAME}
        ${PROJECT_NAME}_core_widgets
        ${OpenCV_LIBS}
    )

    target_include_directories(kenning_yolact_gui_node PUBLIC
        ${OpenCV_INCLUDE_DIRS}
    )

    ament_target_dependencies(kenning_yolact_gui_node
        kenning_computer_vision_msgs
        rclcpp
        rclcpp_components
        sensor_msgs
    )

    rclcpp_components_register_nodes(kenning_yolact_gui_node "gui_node::KenningYolactGuiComponent")

    install(TARGETS
        kenning_yolact_gui_node
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install (FILES examples/kenning-instance-segmentation/kenning-instance-segmentation.py
        DESTINATION share/${PROJECT_NAME}
    )

    install (FILES examples/kenning-instance-segmentation/kenning-instance-segmentation-cpu.py
        DESTINATION share/${PROJECT_NAME}
    )
endif(BUILD_KENNING_YOLACT_DEMO)

#######################
# SamplePublisherComponent
#######################
add_library(sample_publish_node SHARED
    src/samples/sample_publisher_node.cpp
)

target_link_libraries(sample_publish_node
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
)

target_include_directories(sample_publish_node PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(sample_publish_node
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    std_srvs
)

rclcpp_components_register_nodes(sample_publish_node "gui_node::SamplePublisherComponent")

#######################
# Install
#######################
install(TARGETS
    sample_gui_node
    sample_publish_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES launch/sample_launch.py
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

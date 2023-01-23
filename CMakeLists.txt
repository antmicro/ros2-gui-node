cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)

project(gui_node VERSION 0.0.1)

set(GUI_NODE_SRC
    src/main.cpp
)

add_executable(${PROJECT_NAME}
    ${GUI_NODE_SRC}
)

add_custom_target(format
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-tidy -format-style=file -p ${CMAKE_BINARY_DIR} -fix"
    COMMAND bash -c "find ${CMAKE_SOURCE_DIR}/src -iname \"*.cpp\" -o -iname \"*.hpp\" | xargs clang-format --style=file -i"
    VERBATIM
)

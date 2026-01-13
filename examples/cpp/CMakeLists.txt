cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(example)
set(SDK_INSTALL_PATH /opt/galbot/)
include(${SDK_INSTALL_PATH}/galbot_sdk/common.cmake)

add_executable(planner_example planner_example.cpp)
target_include_directories(planner_example PRIVATE ${EMBOSA_SDK_INCLUDES})
target_link_directories(planner_example PRIVATE ${EMBOSA_SDK_LIB_DIR})
target_link_libraries(planner_example PRIVATE ${EMBOSA_SDK_LIBS})
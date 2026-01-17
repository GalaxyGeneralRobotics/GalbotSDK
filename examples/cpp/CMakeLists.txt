cmake_minimum_required(VERSION 3.16)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(example)
set(SDK_INSTALL_PATH /opt/galbot/)
include(${SDK_INSTALL_PATH}/galbot_sdk/common.cmake)

# 对每个代码片段编译一个可执行文件
set(examples_subdir galbot_robot galbot_motion galbot_navigation)
foreach(subdir ${examples_subdir})
    set(CUR_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/src)
    set(EXAMPLE_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/${subdir})
    message(STATUS "CUR_SRC_DIR: ${CUR_SRC_DIR}")

    file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CUR_SRC_DIR}/*.cpp)
    foreach(src_file ${SRC_FILES})
        get_filename_component(example_name ${src_file} NAME_WE)
        set(target_name ${subdir}_${example_name})

        add_executable(${target_name} ${src_file})
        target_include_directories(${target_name}
            PRIVATE ${EMBOSA_SDK_INCLUDES}
        )
        target_link_directories(${target_name}
            PRIVATE ${EMBOSA_SDK_LIB_DIR}
        )
        target_link_libraries(${target_name}
            PRIVATE ${EMBOSA_SDK_LIBS}
        )
        set_target_properties(${target_name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${EXAMPLE_BIN_DIR}
            RUNTIME_OUTPUT_DIRECTORY_DEBUG ${EXAMPLE_BIN_DIR}
            RUNTIME_OUTPUT_DIRECTORY_RELEASE ${EXAMPLE_BIN_DIR}
        )
    endforeach()
endforeach()

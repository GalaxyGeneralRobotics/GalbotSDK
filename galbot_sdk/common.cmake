cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(CMAKE_C_FLAGS "-Wl,--whole-archive -lm -lpthread -ldl -lrt -Wl,--no-whole-archive -fPIC")
# set(CMAKE_CXX_FLAGS "-Wl,--whole-archive -lm -lpthread -ldl -lrt -Wl,--no-whole-archive -fPIC")

get_filename_component(COMMON_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(EMBOSA_SDK_ROOT_DIR ${COMMON_DIR}/${TARGET_PLAT_GCC}/)
set(EMBOSA_SDK_INCLUDES ${EMBOSA_SDK_ROOT_DIR}/include/ ${EMBOSA_SDK_ROOT_DIR}/include/galbot_sdk)
set(EMBOSA_SDK_LIB_DIR ${EMBOSA_SDK_ROOT_DIR}/lib/)
set(EMBOSA_SDK_LIBS 
  galbot_g1_sdk tf2_proto fastcdr fastrtps boost_thread spdlog
  tinyxml2 foonathan_memory-0.7.3 ssl crypto
  embosa core_proto aphropm_proto singorix_proto 
  tf2_proto sensor_proto spatial_proto navigation_proto
  protobuf rt z dl pthread embosa_basic_interface tf2_base tf2_embosa
  opencv_core opencv_imgproc  opencv_imgcodecs png)

message(STATUS EMBOSA_SDK_ROOT_DIR:${EMBOSA_SDK_ROOT_DIR})
message(STATUS EMBOSA_SDK_INCLUDES:${EMBOSA_SDK_INCLUDES})
message(STATUS EMBOSA_SDK_LIB_DIR:${EMBOSA_SDK_LIB_DIR})
message(STATUS EMBOSA_SDK_LIBS:${EMBOSA_SDK_LIBS})
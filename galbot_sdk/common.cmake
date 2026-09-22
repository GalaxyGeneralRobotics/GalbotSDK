cmake_minimum_required(VERSION 3.16)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(CMAKE_C_FLAGS "-Wl,--whole-archive -lm -lpthread -ldl -lrt -Wl,--no-whole-archive -fPIC")
# set(CMAKE_CXX_FLAGS "-Wl,--whole-archive -lm -lpthread -ldl -lrt -Wl,--no-whole-archive -fPIC")

get_filename_component(COMMON_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(EMBOSA_SDK_ROOT_DIR ${COMMON_DIR}/${TARGET_PLAT_GCC}/)
get_filename_component(EMBOSA_INSTALL_ROOT "${COMMON_DIR}" DIRECTORY)

set(EMBOSA_SDK_INCLUDES
  ${EMBOSA_SDK_ROOT_DIR}/include/
  ${EMBOSA_SDK_ROOT_DIR}/include/galbot_sdk
  ${EMBOSA_SDK_ROOT_DIR}/include/eigen3
  ${EMBOSA_SDK_ROOT_DIR}/include/pcl-1.13)
set(EMBOSA_SDK_LIB_DIR ${EMBOSA_SDK_ROOT_DIR}/lib/)
set(EMBOSA_SDK_LIBS 
galbot_sdk fastcdr fastrtps boost_thread spdlog
  tinyxml2 foonathan_memory-0.7.3 ssl crypto
  embosa protobuf rt z dl pthread embosa_basic_interface tf2_base tf2_embosa
  opencv_core opencv_imgproc  opencv_imgcodecs png jpeg
  pcl_common gomp)

set(THIRDPARTY_RPATH_LINK "-Wl,-rpath-link,${EMBOSA_SDK_LIB_DIR}")
set(EMBOSA_MMAPI_RPATH_LINKS "")
set(EMBOSA_TOOLCHAIN_RPATH_LINKS "")
if(TARGET_PLAT_GCC STREQUAL "linux-aarch64-gcc940")
  list(APPEND EMBOSA_MMAPI_RPATH_LINKS
    "${EMBOSA_INSTALL_ROOT}/deps/jetson-mmapi-orin/usr/lib/aarch64-linux-gnu/tegra"
    "${EMBOSA_INSTALL_ROOT}/deps/jetson-mmapi-orin/usr/lib/aarch64-linux-gnu"
    "${EMBOSA_INSTALL_ROOT}/deps/jetson-mmapi-orin/usr/lib/aarch64-linux-gnu/tegra-egl")
elseif(TARGET_PLAT_GCC STREQUAL "linux-aarch64-gcc1330")
  list(APPEND EMBOSA_MMAPI_RPATH_LINKS
    "${EMBOSA_INSTALL_ROOT}/deps/jetson-mmapi-thor/usr/lib/aarch64-linux-gnu/nvidia"
    "${EMBOSA_INSTALL_ROOT}/deps/jetson-mmapi-thor/usr/lib/aarch64-linux-gnu")
  # libatomic.so.1 位于 toolchain lib64，libopencv_core.so 的二级依赖
  # libGL.so.1/libGLX.so.0/libGLdispatch.so.0 位于 toolchain lib。
  # rpath-link 只用于链接阶段解析 DT_NEEDED，不会写入最终程序的运行时路径。
  list(APPEND EMBOSA_TOOLCHAIN_RPATH_LINKS
    "/opt/galbot/toolchain/gcc13.3-aarch64-ubuntu2404-gnu/aarch64-linux-gnu/lib64"
    "/opt/galbot/toolchain/gcc13.3-aarch64-ubuntu2404-gnu/aarch64-linux-gnu/lib")
endif()
foreach(extra_link_dir ${EMBOSA_MMAPI_RPATH_LINKS})
  if(EXISTS "${extra_link_dir}")
    string(APPEND THIRDPARTY_RPATH_LINK " -Wl,-rpath-link,${extra_link_dir}")
  endif()
endforeach()
foreach(extra_link_dir ${EMBOSA_TOOLCHAIN_RPATH_LINKS})
  if(EXISTS "${extra_link_dir}")
    string(APPEND THIRDPARTY_RPATH_LINK " -Wl,-rpath-link,${extra_link_dir}")
  endif()
endforeach()
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${THIRDPARTY_RPATH_LINK}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${THIRDPARTY_RPATH_LINK}")

message(STATUS EMBOSA_SDK_ROOT_DIR:${EMBOSA_SDK_ROOT_DIR})
message(STATUS EMBOSA_INSTALL_ROOT:${EMBOSA_INSTALL_ROOT})
message(STATUS EMBOSA_SDK_INCLUDES:${EMBOSA_SDK_INCLUDES})
message(STATUS EMBOSA_SDK_LIB_DIR:${EMBOSA_SDK_LIB_DIR})
message(STATUS EMBOSA_SDK_LIBS:${EMBOSA_SDK_LIBS})
message(STATUS THIRDPARTY_RPATH_LINK:${THIRDPARTY_RPATH_LINK})

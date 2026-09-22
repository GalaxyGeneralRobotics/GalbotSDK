# 运行可执行文件的系统平台
set(CMAKE_SYSTEM_NAME Linux)
# cpu架构
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(toolchain_target_plat gcc940-aarch64-ubuntu2004-gnu)
# 编译链地址
set(toolchain /opt/galbot/toolchain/${toolchain_target_plat})

# C/C++编译链
set(CMAKE_C_COMPILER ${toolchain}/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${toolchain}/bin/aarch64-linux-gnu-g++)
# 设置连接器搜索地址
set(CMAKE_SYSROOT ${toolchain}/aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH ${toolchain}/aarch64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# specify common variable
set(TARGET_PLAT linux-aarch64)
set(TARGET_PLAT_GCC linux-aarch64-gcc940)

message(STATUS "toolchain: ${TARGET_PLAT_GCC}")

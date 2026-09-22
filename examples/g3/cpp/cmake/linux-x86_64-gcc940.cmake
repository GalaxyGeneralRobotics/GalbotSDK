# 运行可执行文件的系统平台
set(CMAKE_SYSTEM_NAME Linux)
# cpu架构
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(toolchain_target_plat gcc940-x86_64-ubuntu2004-gnu)
# 编译链地址
set(toolchain /opt/galbot/toolchain/${toolchain_target_plat})

# C/C++编译链
set(CMAKE_C_COMPILER ${toolchain}/bin/x86_64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${toolchain}/bin/x86_64-linux-gnu-g++)
# 设置连接器搜索地址
set(CMAKE_SYSROOT ${toolchain}/x86_64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH ${toolchain}/x86_64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# specify common variable
set(TARGET_PLAT linux-x86_64)
set(TARGET_PLAT_GCC linux-x86_64-gcc940)
set(GCOV ${toolchain}/x86_64-linux-gnu-gcov)

message(STATUS "toolchain: ${TARGET_PLAT_GCC}")

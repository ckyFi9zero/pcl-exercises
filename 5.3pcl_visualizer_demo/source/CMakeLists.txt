cmake_minimum_required(VERSION 3.5 FATAL_ERROR)  # 升级 CMake 版本
project(pcl_visualizer_viewports)

# 查找 PCL 和 Boost 依赖
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization features)  # 关键修改
find_package(Boost REQUIRED COMPONENTS thread system)            # 添加 Boost 依赖
find_package(Threads REQUIRED)                                  # 系统线程库

# 包含目录和定义
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 创建可执行文件
add_executable(pcl_visualizer_demo pcl_visualizer_demo.cpp)

# 链接所有必要的库
target_link_libraries(pcl_visualizer_demo
    ${PCL_LIBRARIES}
    ${Boost_THREAD_LIBRARIES}  # Boost 线程库
    ${Boost_SYSTEM_LIBRARIES}  # Boost 系统库
    Threads::Threads           # 系统线程库 (pthread)
)
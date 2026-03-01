cmake_minimum_required(VERSION 3.5)  # 使用现代 CMake 版本
project(narf_feature_extraction)

# 查找 PCL，并明确指定所需组件
find_package(PCL REQUIRED COMPONENTS
    common
    io
    visualization
    features
    keypoints
    range_image
)

# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})

# 添加编译定义
add_definitions(${PCL_DEFINITIONS})

# 创建可执行文件
add_executable(narf_feature_extraction narf_feature_extraction.cpp)

# 链接 PCL 库
target_link_libraries(narf_feature_extraction ${PCL_LIBRARIES})

# (可选) 如果你使用 C++11 或更高标准
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
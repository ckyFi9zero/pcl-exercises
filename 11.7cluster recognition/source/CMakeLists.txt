cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# 启用 CMP0074：支持 XXX_ROOT 变量（可选，避免 FLANN_ROOT 警告）
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

project(vfh_cluster_classifier)

# 查找 PCL（包含 visualization 用于可视化）
find_package(PCL REQUIRED COMPONENTS 
    common 
    io 
    kdtree 
    features 
    visualization  # 用于 pcl::visualization::PCLVisualizer
)

# 查找 HDF5（C 接口，必须！）
find_package(HDF5 REQUIRED COMPONENTS C)

# 查找 FLANN（用于 KNN 搜索）
find_package(FLANN REQUIRED)

# 包含目录
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${HDF5_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)

# 添加定义
add_definitions(${PCL_DEFINITIONS})

# 可执行文件：build_tree（构建 FLANN 索引树）
add_executable(build_tree build_tree.cpp)
target_link_libraries(build_tree
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${HDF5_C_LIBRARIES}   # 👈 关键：链接 HDF5 C 库
)

# 可执行文件：nearest_neighbors（最近邻搜索）
add_executable(nearest_neighbors nearest_neighbors.cpp)
target_link_libraries(nearest_neighbors
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${HDF5_C_LIBRARIES}   # 👈 同样需要链接 HDF5
)
cmake_minimum_required(VERSION 3.10)
project(moment_of_inertia)

# 查找 PCL（必须包含 visualization，因为它依赖 Boost）
find_package(PCL REQUIRED COMPONENTS common io visualization features)

# 查找 Boost（显式要求 thread 和 system）
find_package(Boost REQUIRED COMPONENTS thread system)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(moment_of_inertia moment_of_inertia.cpp)

# 链接库：PCL + Boost + pthread
target_link_libraries(moment_of_inertia
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}   # 👈 添加 Boost 库（thread + system）
    pthread              # 👈 线程底层支持
)

# 包含头文件
target_include_directories(moment_of_inertia PRIVATE ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_compile_definitions(moment_of_inertia PRIVATE ${PCL_DEFINITIONS})
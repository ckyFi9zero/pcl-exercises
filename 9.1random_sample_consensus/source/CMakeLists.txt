cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(random_sample_consensus)

find_package(PCL 1.2 REQUIRED)
find_package(Threads REQUIRED)  # 添加这行
find_package(Boost REQUIRED COMPONENTS thread system)  # 添加这行

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (random_sample_consensus random_sample_consensus.cpp)
# 链接库
target_link_libraries(random_sample_consensus 
    ${PCL_LIBRARIES} 
    ${Boost_THREAD_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY}
    Threads::Threads)
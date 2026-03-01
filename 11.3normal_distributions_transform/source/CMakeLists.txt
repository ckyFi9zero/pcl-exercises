cmake_minimum_required(VERSION 3.10)
project(normal_distributions_transform)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 强制使用多线程、动态链接
set(Boost_USE_STATIC_LIBS OFF)        # ❗关键：不要静态库
set(Boost_USE_STATIC_RUNTIME OFF)
set(Boost_USE_MULTITHREADED ON)

# 查找 Boost（必须组件）
find_package(Boost REQUIRED COMPONENTS
    thread
    system
    chrono
    date_time
    atomic
)

# ✅ 调试：打印 Boost 是否真被找到
message(STATUS "✅ Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "✅ Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "✅ Boost_VERSION: ${Boost_VERSION}")

# 检查关键库是否存在（防止“假成功”）
if(NOT Boost_THREAD_FOUND)
    message(FATAL_ERROR "❌ Boost thread component not found! Run: sudo apt install libboost-thread-dev")
endif()

if(NOT Boost_SYSTEM_FOUND)
    message(FATAL_ERROR "❌ Boost system component not found! Run: sudo apt install libboost-system-dev")
endif()

# 查找 PCL
find_package(PCL REQUIRED)
message(STATUS "✅ PCL_LIBRARIES: ${PCL_LIBRARIES}")

# 查找系统线程（pthread）
find_package(Threads REQUIRED)

# 包含头文件
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(normal_distributions_transform normal_distributions_transform.cpp)

# ✅ 推荐：使用 Imported Targets（更可靠）
target_link_libraries(normal_distributions_transform
    ${PCL_LIBRARIES}
    Boost::thread
    Boost::system
    Boost::chrono
    Boost::date_time
    Boost::atomic
    Threads::Threads
)

# ❌ 不推荐：${Boost_LIBRARIES} 有时顺序不对或包含静态库
# 改用上面的 Boost::xxx 更安全
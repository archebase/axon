cmake_minimum_required(VERSION 3.12)
project(axon_ros1_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (catkin)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    topic_tools
    message_generation
)

# Find nlohmann/json
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(nlohmann_json)

# Build the plugin library (direct C interface, no external ABI header)
add_library(axon_ros1_plugin SHARED
    src/ros1_plugin.cpp
    src/ros1_subscription_wrapper.cpp
    src/ros1_plugin_export.cpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

target_include_directories(axon_ros1_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(axon_ros1_plugin
    ${catkin_LIBRARIES}
    nlohmann_json::nlohmann_json
)

# Export all symbols
target_compile_options(axon_ros1_plugin PRIVATE -fvisibility=default)

# Catkin package setup
catkin_package(
    LIBRARIES axon_ros1_plugin
    CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs topic_tools
)

# Install
install(TARGETS axon_ros1_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

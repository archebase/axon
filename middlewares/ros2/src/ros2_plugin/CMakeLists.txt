cmake_minimum_required(VERSION 3.12)
project(axon_ros2_plugin)

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

# Coverage support
option(AXON_ENABLE_COVERAGE "Enable coverage instrumentation" OFF)
if(AXON_ENABLE_COVERAGE)
    add_compile_options(--coverage -g -O0)
    add_link_options(--coverage)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# Find nlohmann/json
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(nlohmann_json)

# Build the plugin library (direct C interface, no external ABI header)
add_library(axon_ros2_plugin SHARED
    src/ros2_plugin.cpp
    src/ros2_subscription_wrapper.cpp
    src/ros2_plugin_export.cpp
)

target_include_directories(axon_ros2_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(axon_ros2_plugin
    rclcpp::rclcpp
    nlohmann_json::nlohmann_json
)

# Export all symbols
target_compile_options(axon_ros2_plugin PRIVATE -fvisibility=default)

# For ROS2 type support macros - we need to link against message libraries
# This ensures type support symbols are available
if(TARGET std_msgs__rosidl_typesupport_cpp)
    target_link_libraries(axon_ros2_plugin PRIVATE std_msgs__rosidl_typesupport_cpp)
endif()

# Install
install(TARGETS axon_ros2_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib/axon/plugins
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# =============================================================================
# Tests
# =============================================================================
if(AXON_BUILD_TESTS)
  find_package(ament_cmake_gtest REQUIRED)

  # Discover test files automatically
  file(GLOB TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp")

  foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)

    ament_add_gtest(${test_name} ${test_source})

    target_link_libraries(${test_name}
      axon_ros2_plugin
      rclcpp::rclcpp
    )

    # Use threads for tests
    target_link_libraries(${test_name} Threads::Threads)

    # Add test labels
    set_target_properties(${test_name} PROPERTIES
      LABELS "unit;ros2_plugin"
    )
  endforeach()
endif()

ament_package()

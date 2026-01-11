# Performance Test CMake Configuration
# These tests measure recording performance under load

cmake_minimum_required(VERSION 3.15)

find_package(Threads REQUIRED)

# =============================================================================
# Synthetic Publisher - generates test data
# =============================================================================

add_executable(synthetic_publisher synthetic_publisher.cpp)

target_compile_definitions(synthetic_publisher PRIVATE
    $<$<BOOL:${AXON_ROS1}>:AXON_ROS1>
    $<$<BOOL:${AXON_ROS2}>:AXON_ROS2>
)

# ROS 1 dependencies
if(AXON_ROS1)
    target_link_libraries(synthetic_publisher
        ${catkin_LIBRARIES}
        Threads::Threads
    )
    target_include_directories(synthetic_publisher PRIVATE
        ${catkin_INCLUDE_DIRS}
    )
endif()

# ROS 2 dependencies - use modern CMake targets (ament_target_dependencies is deprecated in Rolling)
if(AXON_ROS2)
    target_link_libraries(synthetic_publisher
        Threads::Threads
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
    )
endif()

# =============================================================================
# Performance Test Node - measures CPU, memory, drop rate
# =============================================================================

add_executable(perf_test_node perf_test_node.cpp)

target_compile_definitions(perf_test_node PRIVATE
    $<$<BOOL:${AXON_ROS1}>:AXON_ROS1>
    $<$<BOOL:${AXON_ROS2}>:AXON_ROS2>
)

# ROS 1 dependencies
if(AXON_ROS1)
    target_link_libraries(perf_test_node
        ${catkin_LIBRARIES}
        Threads::Threads
    )
    target_include_directories(perf_test_node PRIVATE
        ${catkin_INCLUDE_DIRS}
    )
endif()

# ROS 2 dependencies - use modern CMake targets (ament_target_dependencies is deprecated in Rolling)
if(AXON_ROS2)
    target_link_libraries(perf_test_node
        Threads::Threads
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
    )
endif()

# =============================================================================
# Install targets
# =============================================================================

install(TARGETS
    synthetic_publisher
    perf_test_node
    DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Test configuration (not run in CI by default - manual execution)
# =============================================================================

# These tests require the full ROS environment and external processes
# They should be run manually using the provided shell scripts

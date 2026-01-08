cmake_minimum_required(VERSION 3.10)
project(ros2_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ============================================================================
# Generate service interfaces
# ============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CachedRecordingConfig.srv"
  "srv/IsRecordingReady.srv"
  "srv/RecordingControl.srv"
  "srv/RecordingStatus.srv"
)

# ============================================================================
# Unified ros2_plugin library (libros2_plugin_lib.so)
# ============================================================================
add_library(${PROJECT_NAME}_lib SHARED
  src/subscriber.cpp
  src/service_adapter.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_lib PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../ros_plugin/core
  /home/xlw/Downloads/ros_plugin/core
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
)

# Link against generated interfaces
target_link_libraries(${PROJECT_NAME}_lib
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install unified library
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export unified library
ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

# ============================================================================
# Install headers
# ============================================================================
install(DIRECTORY include/
  DESTINATION include
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(axon_recorder_plugin)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  std_msgs
  rosserial_server
  topic_tools
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ../../../../../core/axon_utils  # For axon_utils headers
)

# Generate service messages
add_service_files(
  FILES
  CachedRecordingConfig.srv
  IsRecordingReady.srv
  RecordingControl.srv
  RecordingStatus.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib message_runtime std_msgs rosserial_server topic_tools
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/recorder_service_adapter_ros1.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ../../../../../core/axon_utils
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

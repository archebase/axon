cmake_minimum_required(VERSION 3.0)
project(ros1_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  std_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Create shared library
add_library(${PROJECT_NAME} SHARED
  src/subscriber.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

# Catkin package
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS roscpp topic_tools std_msgs
)

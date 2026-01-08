cmake_minimum_required(VERSION 3.0)
project(subscriber_test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable - unified version (works with both ROS1 and ROS2)
add_executable(subscriber_test subscriber_test_unified.cpp plugin_interface.hpp)

# Link against dl library for dynamic loading
target_link_libraries(subscriber_test ${CMAKE_DL_LIBS})

# Install target
install(TARGETS subscriber_test DESTINATION bin)

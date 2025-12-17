cmake_minimum_required(VERSION 3.15)
project(axon)

# Detect ROS version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  # Try to detect from environment
  if(DEFINED ENV{ROS_DISTRO})
    set(ROS_VERSION 1)
  else()
    set(ROS_VERSION 2)
  endif()
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(PkgConfig REQUIRED)

# Find Arrow C++
pkg_check_modules(ARROW REQUIRED arrow)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Build Rust library (C FFI)
set(C_FFI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c)
set(RUST_TARGET_DIR ${C_FFI_DIR}/target)
set(RUST_BUILD_TYPE release)

# Custom target to build Rust library
add_custom_target(cargo_build
    COMMAND cargo build --${RUST_BUILD_TYPE} --manifest-path ${C_FFI_DIR}/Cargo.toml
    WORKING_DIRECTORY ${C_FFI_DIR}
    COMMENT "Building Rust bridge library..."
)

# Determine library extension
if(APPLE)
    set(LIB_EXT ".dylib")
elseif(UNIX)
    set(LIB_EXT ".so")
else()
    set(LIB_EXT ".dll")
endif()

# Import Rust library
set(RUST_LIB_PATH ${RUST_TARGET_DIR}/${RUST_BUILD_TYPE}/liblance_writer_bridge${LIB_EXT})
add_library(lance_bridge_rust SHARED IMPORTED)
set_target_properties(lance_bridge_rust PROPERTIES
    IMPORTED_LOCATION ${RUST_LIB_PATH}
)
add_dependencies(lance_bridge_rust cargo_build)

# ROS 1 Configuration
if(ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
    )
    
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
    )
    
    include_directories(
        include
        ${C_FFI_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${ARROW_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
    )
    
    # Core library
    add_library(${PROJECT_NAME}_core
        cpp/src/core/arrow_builder.cpp
        cpp/src/core/batch_manager.cpp
        cpp/src/core/message_converter.cpp
        cpp/src/core/message_introspection.cpp
        cpp/src/core/schema_merger.cpp
        cpp/src/core/config_parser.cpp
        ros/axon_common/src/message_registry.cpp
        ros/axon_common/src/ros_interface.cpp
        ros/axon_ros1/src/ros1_interface.cpp
        ros/axon_ros1/src/ros1_introspection.cpp
        ros/axon_ros1/src/message_factory.cpp
        ros/axon_ros1/src/register_common_messages.cpp
    )
    
    target_link_libraries(${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
        ${ARROW_LIBRARIES}
        yaml-cpp
        lance_bridge_rust
    )
    
    target_compile_definitions(${PROJECT_NAME}_core PRIVATE ROS_VERSION_1)
    
    # ROS 1 Node
    add_executable(axon_node
        ros/axon_ros1/src/recorder_node.cpp
        ros/axon_ros1/src/recorder_service.cpp
    )
    
    target_link_libraries(axon_node
        ${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
    )
    
    add_dependencies(axon_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    
    # Install
    install(TARGETS axon_node ${PROJECT_NAME}_core
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    
    install(DIRECTORY ros/axon_ros1/launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
    )
    
    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )

# ROS 2 Configuration
else()
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    
    include_directories(
        include
        ${C_FFI_DIR}/include
        ${ARROW_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
    )
    
    # Core library
    add_library(${PROJECT_NAME}_core
        cpp/src/core/arrow_builder.cpp
        cpp/src/core/batch_manager.cpp
        cpp/src/core/message_converter.cpp
        cpp/src/core/message_introspection.cpp
        cpp/src/core/schema_merger.cpp
        cpp/src/core/config_parser.cpp
        ros/axon_common/src/message_registry.cpp
        ros/axon_common/src/ros_interface.cpp
        ros/axon_ros2/src/ros2_interface.cpp
        ros/axon_ros2/src/ros2_introspection.cpp
    )
    
    ament_target_dependencies(${PROJECT_NAME}_core
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
    )
    
    target_link_libraries(${PROJECT_NAME}_core
        ${ARROW_LIBRARIES}
        yaml-cpp
        lance_bridge_rust
    )
    
    target_compile_definitions(${PROJECT_NAME}_core PRIVATE ROS_VERSION_2)
    
    # ROS 2 Node
    add_executable(axon_node
        ros/axon_ros2/src/recorder_node.cpp
        ros/axon_ros2/src/recorder_service.cpp
    )
    
    ament_target_dependencies(axon_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
    )
    
    target_link_libraries(axon_node
        ${PROJECT_NAME}_core
    )
    
    rosidl_target_interfaces(axon_node
        ${PROJECT_NAME} rosidl_typesupport_cpp
    )
    
    # Install
    install(TARGETS axon_node ${PROJECT_NAME}_core
        DESTINATION lib/${PROJECT_NAME}
    )
    
    install(DIRECTORY ros/axon_ros2/launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
    
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
        
        # Add C++ unit tests
        enable_testing()
        add_subdirectory(test)
    endif()
    
    ament_package()
endif()

# Add test support for ROS 1 as well
if(ROS_VERSION EQUAL 1 AND BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

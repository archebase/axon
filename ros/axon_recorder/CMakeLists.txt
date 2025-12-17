cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project paths
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(CPP_SRC_DIR ${PROJECT_ROOT}/cpp/axon_arrow)
set(C_FFI_DIR ${PROJECT_ROOT}/c)

# Detect ROS version and set appropriate macro
if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} EQUAL 1)
        message(STATUS "Building for ROS 1")
        add_definitions(-DAXON_ROS1)
        set(AXON_ROS_VERSION 1)
    elseif($ENV{ROS_VERSION} EQUAL 2)
        message(STATUS "Building for ROS 2")
        add_definitions(-DAXON_ROS2)
        set(AXON_ROS_VERSION 2)
    endif()
else()
    message(WARNING "ROS_VERSION not set, defaulting to ROS 1")
    add_definitions(-DAXON_ROS1)
    set(AXON_ROS_VERSION 1)
endif()

# Find Arrow
find_package(Arrow REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Find pkg-config for additional dependencies
find_package(PkgConfig REQUIRED)

# ============================================================================
# Build Rust C FFI Library
# ============================================================================
set(RUST_TARGET_DIR ${C_FFI_DIR}/target)
set(RUST_BUILD_TYPE release)

# Determine library extension based on platform
if(APPLE)
    set(LIB_EXT ".dylib")
elseif(UNIX)
    set(LIB_EXT ".so")
else()
    set(LIB_EXT ".dll")
endif()

# Custom target to build Rust library
add_custom_target(cargo_build
    COMMAND cargo build --${RUST_BUILD_TYPE} --manifest-path ${C_FFI_DIR}/Cargo.toml
    WORKING_DIRECTORY ${C_FFI_DIR}
    COMMENT "Building Rust Lance bridge library..."
)

# Import Rust library as shared library
set(RUST_LIB_PATH ${RUST_TARGET_DIR}/${RUST_BUILD_TYPE}/liblance_writer_bridge${LIB_EXT})
add_library(lance_bridge_rust SHARED IMPORTED)
set_target_properties(lance_bridge_rust PROPERTIES
    IMPORTED_LOCATION ${RUST_LIB_PATH}
)
add_dependencies(lance_bridge_rust cargo_build)

# ============================================================================
# Build Core C++ Library (from cpp/axon_arrow)
# ============================================================================
# Include axon_arrow as a subdirectory - this builds it as part of this project
# Tests are disabled here since they're run separately via cpp/Makefile
set(AXON_ARROW_BUILD_TESTS OFF CACHE BOOL "Disable axon_arrow tests in ROS build" FORCE)
add_subdirectory(${CPP_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_arrow)

# Add the C FFI include directory and link the Rust bridge
target_include_directories(axon_arrow PUBLIC
    ${C_FFI_DIR}/include
)
target_link_libraries(axon_arrow PUBLIC
    lance_bridge_rust
)

# Alias for backward compatibility
add_library(axon_cpp ALIAS axon_arrow)

# ============================================================================
# ROS 1 specific setup
# ============================================================================
if(AXON_ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        message_generation
    )
    
    # Generate service messages
    add_service_files(
        FILES
        srv/StartRecording.srv
        srv/StopRecording.srv
        srv/UpdateConfig.srv
        srv/GetStatus.srv
    )
    
    generate_messages(
        DEPENDENCIES
        std_msgs
    )
    
    catkin_package(
        INCLUDE_DIRS src
        LIBRARIES axon_recorder_lib
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime
    )
    
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Arrow_INCLUDE_DIRS}
        src
        ${CPP_SRC_DIR}
        ${C_FFI_DIR}/include
    )
    
    # Auto-discover recorder library sources
    file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    )
    # Exclude node main file from library
    list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*recorder_node\\.cpp$")
    
    # Recorder Library
    add_library(axon_recorder_lib ${RECORDER_LIB_SOURCES})
    
    target_link_libraries(axon_recorder_lib
        ${catkin_LIBRARIES}
        Arrow::arrow_shared
        axon_cpp
    )
    
    add_dependencies(axon_recorder_lib cargo_build)
    
    # Executable
    add_executable(axon_recorder_node
        src/recorder_node.cpp
    )
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
        ${catkin_LIBRARIES}
    )
    
    add_dependencies(axon_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
    
    install(DIRECTORY srv/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
        FILES_MATCHING PATTERN "*.srv"
    )
    
    # Install Rust library
    install(FILES ${RUST_LIB_PATH}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    
# ============================================================================
# ROS 2 specific setup
# ============================================================================
elseif(AXON_ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    
    # Generate service messages
    rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/StartRecording.srv"
        "srv/StopRecording.srv"
        "srv/UpdateConfig.srv"
        "srv/GetStatus.srv"
        DEPENDENCIES std_msgs
    )
    
    include_directories(
        ${Arrow_INCLUDE_DIRS}
        src
        ${CPP_SRC_DIR}
        ${C_FFI_DIR}/include
    )
    
    # Auto-discover recorder library sources
    file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    )
    # Exclude node main file from library
    list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*recorder_node\\.cpp$")
    
    # Recorder Library
    add_library(axon_recorder_lib SHARED ${RECORDER_LIB_SOURCES})
    
    ament_target_dependencies(axon_recorder_lib
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
    
    target_link_libraries(axon_recorder_lib
        Arrow::arrow_shared
        axon_cpp
    )
    
    add_dependencies(axon_recorder_lib cargo_build)
    
    # Executable
    add_executable(axon_recorder_node
        src/recorder_node.cpp
    )
    
    ament_target_dependencies(axon_recorder_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
    )
    
    rosidl_target_interfaces(axon_recorder_node
        ${PROJECT_NAME} rosidl_typesupport_cpp
    )
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    
    install(DIRECTORY srv/
        DESTINATION share/${PROJECT_NAME}/srv
        FILES_MATCHING PATTERN "*.srv"
    )
    
    # Install Rust library
    install(FILES ${RUST_LIB_PATH}
        DESTINATION lib
    )
    
    ament_package()
endif()

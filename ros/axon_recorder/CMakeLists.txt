cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

# Allow linking libraries to targets defined in subdirectories
# Required because axon_arrow is built in cpp/axon_arrow but linked here
cmake_policy(SET CMP0079 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Code Formatting with clang-format
# =============================================================================
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake" ${CMAKE_MODULE_PATH})
include(ClangFormat OPTIONAL)

# ============================================================================
# Address Sanitizer (ASAN) Support
# Enable with: cmake -DENABLE_ASAN=ON or set ENABLE_ASAN=1 environment variable
# ============================================================================
option(ENABLE_ASAN "Enable Address Sanitizer for debugging memory issues" OFF)

# Also check environment variable
if(DEFINED ENV{ENABLE_ASAN} AND "$ENV{ENABLE_ASAN}" STREQUAL "1")
    set(ENABLE_ASAN ON)
endif()

if(ENABLE_ASAN)
    message(STATUS "=== ADDRESS SANITIZER ENABLED ===")
    message(STATUS "Building with ASAN for memory debugging")
    
    # ASAN compiler and linker flags
    set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer -g -O1")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
    
    # Also set for axon_arrow subdirectory
    set(AXON_ARROW_ASAN_FLAGS "${ASAN_FLAGS}" CACHE STRING "ASAN flags for axon_arrow" FORCE)
endif()

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

# Add custom config files to suppress Arrow's CMake warnings
# Arrow uses find_package(lz4 CONFIG) which looks for lz4-config.cmake in CMAKE_PREFIX_PATH
# Ubuntu packages don't provide these, so we provide pkg-config wrappers
list(PREPEND CMAKE_PREFIX_PATH 
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/lz4"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/zstd"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/re2"
)

# Find Arrow (our config files prevent "lz4Config.cmake not found" warnings)
find_package(Arrow REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# =============================================================================
# Clang-format targets (before ROS-specific setup)
# =============================================================================
if(COMMAND axon_add_clang_format)
    # Collect all source files for formatting
    file(GLOB_RECURSE AXON_RECORDER_ALL_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    
    # Create format targets (they don't depend on any build target)
    add_custom_target(axon_recorder_format_target)
    axon_add_clang_format(axon_recorder_format_target SOURCE_FILES ${AXON_RECORDER_ALL_SOURCES})
    axon_register_format_target(axon_recorder_format_target)
endif()

# Find pkg-config for additional dependencies
find_package(PkgConfig REQUIRED)

# ============================================================================
# Build Rust C FFI Library
# ============================================================================
# Note: Rust workspace outputs to the root target/ directory, not c/target/
set(RUST_TARGET_DIR ${PROJECT_ROOT}/target)
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
message(STATUS "Rust library path: ${RUST_LIB_PATH}")
message(STATUS "PROJECT_ROOT: ${PROJECT_ROOT}")

add_library(lance_bridge_rust SHARED IMPORTED GLOBAL)
set_target_properties(lance_bridge_rust PROPERTIES
    IMPORTED_LOCATION ${RUST_LIB_PATH}
    IMPORTED_NO_SONAME TRUE
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
# Use PRIVATE to avoid source directory paths in INTERFACE_INCLUDE_DIRECTORIES
target_include_directories(axon_arrow PRIVATE
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
    
    # Build tests (perf tests, integration tests)
    set(AXON_ROS1 TRUE)
    add_subdirectory(test)

# ============================================================================
# ROS 2 specific setup
# ============================================================================
elseif(AXON_ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
    
    # Workaround: Ensure MEMBER_OF_GROUPS is set for rosidl_cmake check
    # Some colcon/catkin_pkg versions don't correctly extract member_of_group from package.xml
    set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages" CACHE INTERNAL "" FORCE)
    
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    
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
        ament_index_cpp
    )
    
    # Link to Rust FFI library - must use full path for shared library
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
        "${RUST_LIB_PATH}"
    )
    
    # Ensure Rust library is built before linking
    add_dependencies(axon_recorder_node cargo_build)
    
    # Link to generated service typesupport (replaces deprecated rosidl_target_interfaces)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(axon_recorder_node "${cpp_typesupport_target}")
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )
    
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )
    
    install(DIRECTORY srv/
        DESTINATION share/${PROJECT_NAME}/srv
        FILES_MATCHING PATTERN "*.srv"
    )
    
    # Install Rust library
    install(FILES ${RUST_LIB_PATH}
        DESTINATION lib
    )
    
    # Build tests (perf tests, integration tests)
    set(AXON_ROS2 TRUE)
    add_subdirectory(test)
    
    ament_package()
endif()

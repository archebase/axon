cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

# Allow linking libraries to targets defined in subdirectories
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
endif()

# Project paths - handle both direct build and colcon build scenarios
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Find axon_mcap directory - check multiple possible locations
if(EXISTS "${PROJECT_ROOT}/cpp/axon_mcap/CMakeLists.txt")
    set(CPP_MCAP_DIR ${PROJECT_ROOT}/cpp/axon_mcap)
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../cpp/axon_mcap/CMakeLists.txt")
    set(CPP_MCAP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp/axon_mcap)
else()
    message(FATAL_ERROR "Could not find axon_mcap directory. Searched:\n"
        "  ${PROJECT_ROOT}/cpp/axon_mcap\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/../../cpp/axon_mcap\n"
        "Please ensure the cpp/axon_mcap directory exists relative to the project root.")
endif()
message(STATUS "Found axon_mcap at: ${CPP_MCAP_DIR}")

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

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# Find Boost for HTTP callback client (Beast requires system and optionally ssl)
find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    message(STATUS "Found Boost: ${Boost_VERSION}")
endif()

# Find pkg-config for compression libraries
find_package(PkgConfig REQUIRED)

# Find compression libraries for MCAP
pkg_check_modules(ZSTD QUIET libzstd)
pkg_check_modules(LZ4 QUIET liblz4)

if(ZSTD_FOUND)
    message(STATUS "Found Zstd: ${ZSTD_VERSION}")
else()
    message(WARNING "Zstd not found - MCAP compression disabled")
endif()

if(LZ4_FOUND)
    message(STATUS "Found LZ4: ${LZ4_VERSION}")
endif()

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

# ============================================================================
# Build MCAP Library (includes config parser)
# ============================================================================
message(STATUS "Building with MCAP backend")
add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)

# ============================================================================
# ROS 1 specific setup
# ============================================================================
if(AXON_ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        topic_tools
        message_generation
    )
    
    # Generate service messages
    # Note: add_service_files looks in srv/ directory by default
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
    
    catkin_package(
        INCLUDE_DIRS src
        LIBRARIES axon_recorder_lib
        CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs nav_msgs geometry_msgs topic_tools message_runtime
    )
    
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        src
        ${CPP_MCAP_DIR}
        ${CPP_MCAP_DIR}/include
    )
    
    # Auto-discover recorder library sources
    file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    )
    # Exclude main entry point from library
    list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
    
    # Recorder Library
    add_library(axon_recorder_lib ${RECORDER_LIB_SOURCES})
    
    target_link_libraries(axon_recorder_lib
        ${catkin_LIBRARIES}
        axon_mcap
        ${Boost_LIBRARIES}
        yaml-cpp
    )
    
    # Executable
    add_executable(axon_recorder_node
        src/main.cpp
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
    
    install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )
    
    install(DIRECTORY srv/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
        FILES_MATCHING PATTERN "*.srv"
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
        "srv/CachedRecordingConfig.srv"
        "srv/IsRecordingReady.srv"
        "srv/RecordingControl.srv"
        "srv/RecordingStatus.srv"
        DEPENDENCIES std_msgs
    )
    
    include_directories(
        src
        ${Boost_INCLUDE_DIRS}
        ${CPP_MCAP_DIR}
        ${CPP_MCAP_DIR}/include
    )
    
    # Auto-discover recorder library sources
    file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    )
    # Exclude main entry point from library
    list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*main\\.cpp$")
    
    # Recorder Library
    add_library(axon_recorder_lib SHARED ${RECORDER_LIB_SOURCES})
    
    ament_target_dependencies(axon_recorder_lib
        rclcpp
        ament_index_cpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
    )
    
    target_link_libraries(axon_recorder_lib
        axon_mcap
        ${Boost_LIBRARIES}
        yaml-cpp
    )
    
    # Add Boost include directories
    target_include_directories(axon_recorder_lib PRIVATE ${Boost_INCLUDE_DIRS})
    
    # Link to generated service typesupport for the library
    rosidl_get_typesupport_target(cpp_typesupport_target_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(axon_recorder_lib "${cpp_typesupport_target_lib}")
    
    # Link compression libraries if available
    if(ZSTD_FOUND)
        target_link_libraries(axon_recorder_lib ${ZSTD_LIBRARIES})
    endif()
    if(LZ4_FOUND)
        target_link_libraries(axon_recorder_lib ${LZ4_LIBRARIES})
    endif()
    
    # Executable
    add_executable(axon_recorder_node
        src/main.cpp
    )
    
    ament_target_dependencies(axon_recorder_node
        rclcpp
        std_msgs
        sensor_msgs
        nav_msgs
        geometry_msgs
        ament_index_cpp
    )
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
    )
    
    # Link to generated service typesupport (replaces deprecated rosidl_target_interfaces)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(axon_recorder_node "${cpp_typesupport_target}")
    
    # Install
    install(TARGETS axon_recorder_lib axon_recorder_node axon_mcap
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
    
    # Build tests (perf tests, integration tests)
    set(AXON_ROS2 TRUE)
    add_subdirectory(test)
    
    ament_package()
endif()

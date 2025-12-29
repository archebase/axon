cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allow linking libraries to targets defined in subdirectories
cmake_policy(SET CMP0079 NEW)

# Suppress warning about unused variables passed by industrial_ci/catkin
# These are only used in ROS1 builds but get passed to ROS2 builds too
set(ignored_variables
    "${CATKIN_INSTALL_INTO_PREFIX_ROOT}"
    "${CATKIN_DEVEL_PREFIX}"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# ============================================================================
# Code Coverage Support (gcov/lcov)
# Enable with: cmake -DENABLE_COVERAGE=ON or set ENABLE_COVERAGE=1 environment variable
# ============================================================================
option(ENABLE_COVERAGE "Enable code coverage instrumentation (gcov)" OFF)

# Also check environment variable
if(DEFINED ENV{ENABLE_COVERAGE} AND "$ENV{ENABLE_COVERAGE}" STREQUAL "1")
    set(ENABLE_COVERAGE ON)
endif()

if(ENABLE_COVERAGE)
    message(STATUS "=== CODE COVERAGE ENABLED ===")
    message(STATUS "Building with coverage instrumentation (gcov)")
    
    # Coverage compiler flags for GCC/Clang
    set(COVERAGE_FLAGS "-fprofile-arcs -ftest-coverage -g -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_FLAGS}")
    
    # Link with gcov library
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
    
    # Ensure debug symbols are included
    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(WARNING "Coverage is most accurate with Debug build. Consider using -DCMAKE_BUILD_TYPE=Debug")
    endif()
endif()

# Project paths - handle both direct build and colcon build scenarios
set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Find axon_mcap directory (fixed location relative to repo root)
set(CPP_MCAP_DIR ${PROJECT_ROOT}/../cpp/axon_mcap)
if(NOT EXISTS "${CPP_MCAP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_mcap at: ${CPP_MCAP_DIR}\n"
        "Please ensure the cpp/axon_mcap directory exists relative to the project root.")
endif()
message(STATUS "Found axon_mcap at: ${CPP_MCAP_DIR}")

# Detect ROS version (definitions applied per-target below, not globally)
if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} EQUAL 1)
        message(STATUS "Building for ROS 1")
        set(AXON_ROS_VERSION 1)
        set(AXON_ROS_DEFINITION "AXON_ROS1")
    elseif($ENV{ROS_VERSION} EQUAL 2)
        message(STATUS "Building for ROS 2")
        set(AXON_ROS_VERSION 2)
        set(AXON_ROS_DEFINITION "AXON_ROS2")
    endif()
else()
    message(WARNING "ROS_VERSION not set, defaulting to ROS 1")
    set(AXON_ROS_VERSION 1)
    set(AXON_ROS_DEFINITION "AXON_ROS1")
endif()

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

# =============================================================================
# nlohmann/json (header-only JSON library for metadata sidecar generation)
# =============================================================================
include(FetchContent)
# DOWNLOAD_EXTRACT_TIMESTAMP requires CMake 3.24+
# Use it conditionally to avoid errors on older CMake (Humble uses 3.22, Noetic uses 3.16)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
  FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
    URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
else()
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
endif()
FetchContent_MakeAvailable(nlohmann_json)

# =============================================================================
# Version header generation from package.xml
# =============================================================================
# Extract version from package.xml
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>" VERSION_MATCH "${PACKAGE_XML_CONTENT}")
set(PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
set(PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
set(PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

message(STATUS "axon_recorder version: ${PROJECT_VERSION}")

# Generate version.hpp from template
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/src/version.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp"
  @ONLY
)

# Find Boost for HTTP callback client (Beast requires system) and logging
find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
if(Boost_FOUND)
    message(STATUS "Found Boost: ${Boost_VERSION}")
endif()

# Find axon_logging directory (fixed location relative to repo root)
set(CPP_LOGGING_DIR ${PROJECT_ROOT}/../cpp/axon_logging)
if(NOT EXISTS "${CPP_LOGGING_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_logging at: ${CPP_LOGGING_DIR}\n"
        "Please ensure the cpp/axon_logging directory exists relative to the project root.")
endif()
message(STATUS "Found axon_logging at: ${CPP_LOGGING_DIR}")

# Find OpenSSL for HTTPS support in HTTP callback client
find_package(OpenSSL REQUIRED)
if(OpenSSL_FOUND)
    message(STATUS "Found OpenSSL: ${OPENSSL_VERSION}")
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

# Propagate coverage flag to axon_mcap subdirectory
# This ensures mcap_validator.cpp and mcap_writer_wrapper.cpp get coverage instrumentation
# when axon_recorder is built with ENABLE_COVERAGE=ON (e.g., in CI ROS tests)
if(ENABLE_COVERAGE)
    set(AXON_MCAP_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_mcap" FORCE)
endif()

add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)

# ============================================================================
# Auto-discover recorder library sources (shared by ROS1 and ROS2)
# ============================================================================
file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
# Exclude main entry point from library
list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

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
        ${CPP_LOGGING_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )
    
    # Recorder Library (sources discovered above)
    add_library(axon_recorder_lib ${RECORDER_LIB_SOURCES})
    
    # Per-target compile definitions (not global add_definitions)
    target_compile_definitions(axon_recorder_lib PRIVATE 
        ${AXON_ROS_DEFINITION}
        BOOST_LOG_DYN_LINK
    )
    
    target_link_libraries(axon_recorder_lib
        ${catkin_LIBRARIES}
        axon_mcap
        ${Boost_LIBRARIES}
        OpenSSL::SSL
        OpenSSL::Crypto
        yaml-cpp
        nlohmann_json::nlohmann_json
    )
    
    # Ensure library waits for message generation (service headers)
    add_dependencies(axon_recorder_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    
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
        ${CPP_LOGGING_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )
    
    # Recorder Library (sources discovered above)
    add_library(axon_recorder_lib SHARED ${RECORDER_LIB_SOURCES})
    
    # Per-target compile definitions (not global add_definitions)
    target_compile_definitions(axon_recorder_lib PRIVATE 
        ${AXON_ROS_DEFINITION}
        BOOST_LOG_DYN_LINK
    )
    
    # Use modern CMake target_link_libraries() instead of deprecated ament_target_dependencies()
    # This is required for ROS2 Rolling compatibility and avoids slow builds/warnings.
    # See: .cursor/rules/ros-dual-support.mdc for details
    target_link_libraries(axon_recorder_lib
        axon_mcap
        ${Boost_LIBRARIES}
        OpenSSL::SSL
        OpenSSL::Crypto
        yaml-cpp
        nlohmann_json::nlohmann_json
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        sensor_msgs::sensor_msgs_library
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
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
    
    target_link_libraries(axon_recorder_node
        axon_recorder_lib
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        sensor_msgs::sensor_msgs_library
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
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

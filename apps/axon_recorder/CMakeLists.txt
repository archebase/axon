cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Allow linking libraries to targets defined in subdirectories
cmake_policy(SET CMP0079 NEW)

# =============================================================================
# Centralized CMake Modules
# =============================================================================
# All Axon modules must use the centralized cmake/Modules directory.
# This ensures consistent configuration across the codebase.
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
else()
    # Compute path to cmake/Modules relative to this directory
    # From apps/axon_recorder/, the repo root is ../..
    set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/Modules")
    if(NOT EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdlib.cmake")
        message(FATAL_ERROR
            "Axon CMake modules not found at: ${_AXON_CMAKE_MODULES_DIR}\n"
            "Please ensure you're building from the Axon repository root."
        )
    endif()
    list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
endif()

# Include required modules (fail if not found)
include(AxonStdlib REQUIRED)
include(AxonCoverage REQUIRED)

# =============================================================================
# Path Configuration
# =============================================================================
# Calculate paths relative to repo root.
# From apps/axon_recorder/, the repo root is ../..

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Use centralized AXON_* paths if available (built from root), otherwise calculate
if(DEFINED AXON_MCAP_DIR AND EXISTS "${AXON_MCAP_DIR}/CMakeLists.txt")
    set(CPP_MCAP_DIR "${AXON_MCAP_DIR}")
else()
    set(CPP_MCAP_DIR "${PROJECT_ROOT}/core/axon_mcap")
endif()

if(DEFINED AXON_LOGGING_DIR AND EXISTS "${AXON_LOGGING_DIR}/CMakeLists.txt")
    set(CPP_LOGGING_DIR "${AXON_LOGGING_DIR}")
else()
    set(CPP_LOGGING_DIR "${PROJECT_ROOT}/core/axon_logging")
endif()

if(DEFINED AXON_UTILS_DIR AND EXISTS "${AXON_UTILS_DIR}/CMakeLists.txt")
    set(CPP_UTILS_DIR "${AXON_UTILS_DIR}")
else()
    set(CPP_UTILS_DIR "${PROJECT_ROOT}/core/axon_utils")
endif()

# Validate required paths
if(NOT EXISTS "${CPP_MCAP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_mcap at: ${CPP_MCAP_DIR}\n"
        "Please ensure the core/axon_mcap directory exists.")
endif()
message(STATUS "Found axon_mcap at: ${CPP_MCAP_DIR}")

if(NOT EXISTS "${CPP_LOGGING_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_logging at: ${CPP_LOGGING_DIR}\n"
        "Please ensure the core/axon_logging directory exists.")
endif()
message(STATUS "Found axon_logging at: ${CPP_LOGGING_DIR}")

if(NOT EXISTS "${CPP_UTILS_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_utils at: ${CPP_UTILS_DIR}\n"
        "Please ensure the core/axon_utils directory exists.")
endif()
message(STATUS "Found axon_utils at: ${CPP_UTILS_DIR}")

# Export to test subdirectory
set(PROJECT_ROOT ${PROJECT_ROOT} CACHE INTERNAL "Axon repo root")
set(CPP_LOGGING_DIR ${CPP_LOGGING_DIR} CACHE INTERNAL "Path to axon_logging")
set(CPP_MCAP_DIR ${CPP_MCAP_DIR} CACHE INTERNAL "Path to axon_mcap")
set(CPP_UTILS_DIR ${CPP_UTILS_DIR} CACHE INTERNAL "Path to axon_utils")

# =============================================================================
# Dependencies
# =============================================================================

find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)

# Compression libraries
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
# nlohmann/json (header-only for metadata sidecar generation)
# =============================================================================
include(FetchContent)
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
# Version Header Generation
# =============================================================================
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9]+)\\.([0-9]+)\\.([0-9]+)</version>" VERSION_MATCH "${PACKAGE_XML_CONTENT}")
set(PROJECT_VERSION_MAJOR ${CMAKE_MATCH_1})
set(PROJECT_VERSION_MINOR ${CMAKE_MATCH_2})
set(PROJECT_VERSION_PATCH ${CMAKE_MATCH_3})
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

message(STATUS "axon_recorder version: ${PROJECT_VERSION}")

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/src/version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp"
    @ONLY
)

# =============================================================================
# Build axon_mcap and axon_utils
# =============================================================================
message(STATUS "Building with MCAP backend")

# Normalize ENABLE_COVERAGE to AXON_ENABLE_COVERAGE (centralized)
# Explicitly set to OFF/empty to prevent cached values from persisting
if(DEFINED ENABLE_COVERAGE)
    set(AXON_ENABLE_COVERAGE ${ENABLE_COVERAGE} CACHE BOOL "Coverage enabled" FORCE)
else()
    set(AXON_ENABLE_COVERAGE OFF CACHE BOOL "Coverage enabled" FORCE)
endif()

# Propagate coverage flag to axon_mcap (must explicitly set OFF to clear cache)
if(AXON_ENABLE_COVERAGE)
    set(AXON_MCAP_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_mcap" FORCE)
else()
    set(AXON_MCAP_ENABLE_COVERAGE OFF CACHE BOOL "Enable coverage for axon_mcap" FORCE)
endif()

add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
add_subdirectory(${CPP_UTILS_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_utils)

# =============================================================================
# Core Plugin Loading Library (No ROS dependencies)
# =============================================================================
# Only compile core utilities: config_parser, http_client, and adapter headers
# Exclude all ROS-specific code (ros_interface, service_adapter, recorder_node, etc.)

set(CORE_SOURCES
    src/common_types.cpp
    src/config_parser.cpp
    src/http_client.cpp
)

include_directories(
    ${Boost_INCLUDE_DIRS}
    src
    ${CPP_MCAP_DIR}
    ${CPP_LOGGING_DIR}
    ${CPP_UTILS_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# Core library (plugin loading mechanism)
add_library(axon_recorder_core STATIC ${CORE_SOURCES})

target_compile_definitions(axon_recorder_core PRIVATE
    BOOST_LOG_DYN_LINK
)

target_link_libraries(axon_recorder_core PUBLIC
    axon_mcap
    axon_utils
    ${Boost_LIBRARIES}
    OpenSSL::SSL
    OpenSSL::Crypto
    yaml-cpp
    nlohmann_json::nlohmann_json
)

target_include_directories(axon_recorder_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CPP_UTILS_DIR}
)

# Apply coverage flags if enabled
if(AXON_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(axon_recorder_core PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(axon_recorder_core PRIVATE --coverage)
    endif()
endif()

if(ZSTD_FOUND)
    target_link_libraries(axon_recorder_core PUBLIC ${ZSTD_LIBRARIES})
endif()
if(LZ4_FOUND)
    target_link_libraries(axon_recorder_core PUBLIC ${LZ4_LIBRARIES})
endif()

# =============================================================================
# Executable Program (with ROS dependencies)
# =============================================================================
# The executable needs ROS support to run, so it includes all ROS-related code

# Detect ROS version
if(DEFINED ROS_VERSION)
    if(ROS_VERSION EQUAL 1)
        set(AXON_ROS1 ON)
        message(STATUS "Building axon_recorder for ROS1")
    elseif(ROS_VERSION EQUAL 2)
        set(AXON_ROS2 ON)
        message(STATUS "Building axon_recorder for ROS2")
    endif()
else()
    # Try to detect ROS automatically
    if(EXISTS "/opt/ros/noetic")
        set(AXON_ROS1 ON)
        message(STATUS "Auto-detected ROS1 (Noetic)")
    elseif(EXISTS "/opt/ros/humble" OR EXISTS "/opt/ros/iron" OR EXISTS "/opt/ros/jazzy" OR EXISTS "/opt/ros/rolling")
        set(AXON_ROS2 ON)
        message(STATUS "Auto-detected ROS2")
    else()
        message(WARNING "ROS not detected. axon_recorder executable will not be built.")
        set(BUILD_EXECUTABLE OFF)
    endif()
endif()

if(NOT DEFINED BUILD_EXECUTABLE)
    set(BUILD_EXECUTABLE ON)
endif()

# Note: ROS service interfaces are generated in middleware plugins, not here.
# apps/axon_recorder must remain independent of ROS.

if(BUILD_EXECUTABLE AND (AXON_ROS1 OR AXON_ROS2))
    # Executable sources (includes ROS-specific code)
    set(EXECUTABLE_SOURCES
        src/main.cpp
        src/recorder_node.cpp
        src/recording_session.cpp
        src/metadata_injector.cpp
        src/topic_manager.cpp
        src/ros_interface.cpp
        src/service_adapter.cpp
        src/recording_service_impl.cpp
        src/ros_introspection.cpp
        src/message_factory.cpp
        src/register_common_messages.cpp
        src/logging/axon_ros_sink.cpp
    )

    # Create executable
    add_executable(axon_recorder ${EXECUTABLE_SOURCES})

    # Include directories for executable
    # Note: For <axon_utils/...> and <axon_mcap/...> includes, we need the parent directory
    # since headers are in core/axon_utils/ and core/axon_mcap/ directly
    get_filename_component(CPP_CORE_DIR ${CPP_UTILS_DIR} DIRECTORY)
    target_include_directories(axon_recorder PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CPP_CORE_DIR}  # For <axon_utils/...> and <axon_mcap/...> includes
        ${CPP_LOGGING_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/generated
    )

    # Compile definitions
    target_compile_definitions(axon_recorder PRIVATE
        BOOST_LOG_DYN_LINK
    )

    if(AXON_ROS1)
        target_compile_definitions(axon_recorder PRIVATE AXON_ROS1)
        find_package(catkin REQUIRED COMPONENTS
            roscpp
            roslib
            std_msgs
            sensor_msgs
            nav_msgs
            geometry_msgs
            topic_tools
        )
        target_link_libraries(axon_recorder PUBLIC
            ${catkin_LIBRARIES}
        )
        target_include_directories(axon_recorder PUBLIC
            ${catkin_INCLUDE_DIRS}
        )
        
        # Link to middleware plugin for service interfaces
        # Service interfaces are generated in middlewares/ros1/src/axon_recorder_plugin
        # The plugin should be built separately and linked here
        set(ROS1_PLUGIN_DIR "${PROJECT_ROOT}/middlewares/ros1/src/axon_recorder_plugin")
        if(EXISTS "${ROS1_PLUGIN_DIR}/CMakeLists.txt")
            add_subdirectory(${ROS1_PLUGIN_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ros1_plugin)
            # Link to the generated message headers
            target_include_directories(axon_recorder PRIVATE
                ${CMAKE_CURRENT_BINARY_DIR}/ros1_plugin/include
            )
        endif()
    elseif(AXON_ROS2)
        target_compile_definitions(axon_recorder PRIVATE AXON_ROS2)
        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(ament_index_cpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        
        target_link_libraries(axon_recorder PUBLIC
            rclcpp::rclcpp
            ament_index_cpp::ament_index_cpp
            ${std_msgs_TARGETS}
            ${sensor_msgs_TARGETS}
            ${nav_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
        )
        
        # Link to middleware plugin for service interfaces
        # Service interfaces are generated in middlewares/ros2/src/ros2_plugin
        # The plugin should be built separately and linked here
        set(ROS2_PLUGIN_DIR "${PROJECT_ROOT}/middlewares/ros2/src/ros2_plugin")
        if(EXISTS "${ROS2_PLUGIN_DIR}/CMakeLists.txt")
            add_subdirectory(${ROS2_PLUGIN_DIR} ${CMAKE_CURRENT_BINARY_DIR}/ros2_plugin)
            # Link to the generated service interfaces
            # rosidl_generate_interfaces creates targets with format: ${PROJECT_NAME}__rosidl_typesupport_cpp
            target_link_libraries(axon_recorder PUBLIC
                ros2_plugin__rosidl_typesupport_cpp
            )
            target_include_directories(axon_recorder PRIVATE
                ${CMAKE_CURRENT_BINARY_DIR}/ros2_plugin/rosidl_generator_cpp
            )
        endif()
    endif()

    # Link libraries
    target_link_libraries(axon_recorder PUBLIC
        axon_recorder_core
        axon_mcap
        axon_utils
        ${Boost_LIBRARIES}
        OpenSSL::SSL
        OpenSSL::Crypto
        yaml-cpp
        nlohmann_json::nlohmann_json
    )

    if(ZSTD_FOUND)
        target_link_libraries(axon_recorder PUBLIC ${ZSTD_LIBRARIES})
    endif()
    if(LZ4_FOUND)
        target_link_libraries(axon_recorder PUBLIC ${LZ4_LIBRARIES})
    endif()

    # Apply coverage flags if enabled
    if(AXON_ENABLE_COVERAGE)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
            target_compile_options(axon_recorder PRIVATE --coverage -fprofile-arcs -ftest-coverage)
            target_link_options(axon_recorder PRIVATE --coverage)
        endif()
    endif()

    # Install executable
    install(TARGETS axon_recorder
        RUNTIME DESTINATION bin
    )
endif()

# Install core library
install(TARGETS axon_recorder_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers (only core headers, no ROS-specific)
install(FILES
    src/common_types.hpp
    src/config_parser.hpp
    src/http_client.hpp
    src/recorder_adapter.hpp
    DESTINATION include/axon_recorder
)

# Build tests (if test directory exists and doesn't require ROS)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
    # Only build tests that don't require ROS
    # Tests will be handled separately if needed
    # add_subdirectory(test)
endif()

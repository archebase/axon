cmake_minimum_required(VERSION 3.12)
project(axon_recorder)

# Export compile commands for clangd/IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Centralized CMake Modules
# =============================================================================
if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
else()
    if(DEFINED AXON_REPO_ROOT)
        set(_AXON_CMAKE_MODULES_DIR "${AXON_REPO_ROOT}/cmake/Modules")
    else()
        set(_AXON_CMAKE_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake/Modules")
    endif()
    if(NOT EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdlib.cmake")
        message(FATAL_ERROR
            "Axon CMake modules not found at: ${_AXON_CMAKE_MODULES_DIR}\n"
            "Please ensure you're building from the Axon repository root."
        )
    endif()
    list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
endif()

include(AxonStdlib REQUIRED)
include(AxonCoverage REQUIRED)

# =============================================================================
# Path Configuration
# =============================================================================
if(DEFINED AXON_REPO_ROOT)
    set(PROJECT_ROOT ${AXON_REPO_ROOT})
else()
    set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../..)
endif()

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

if(NOT EXISTS "${CPP_MCAP_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_mcap at: ${CPP_MCAP_DIR}")
endif()
message(STATUS "Found axon_mcap at: ${CPP_MCAP_DIR}")

if(NOT EXISTS "${CPP_LOGGING_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "Could not find axon_logging at: ${CPP_LOGGING_DIR}")
endif()
message(STATUS "Found axon_logging at: ${CPP_LOGGING_DIR}")

set(PROJECT_ROOT ${PROJECT_ROOT} CACHE INTERNAL "Axon repo root")
set(CPP_LOGGING_DIR ${CPP_LOGGING_DIR} CACHE INTERNAL "Path to axon_logging")
set(CPP_MCAP_DIR ${CPP_MCAP_DIR} CACHE INTERNAL "Path to axon_mcap")

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
# nlohmann/json (header-only)
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
# Build axon_mcap
# =============================================================================
message(STATUS "Building with MCAP backend")

if(DEFINED ENABLE_COVERAGE)
    set(AXON_ENABLE_COVERAGE ${ENABLE_COVERAGE} CACHE BOOL "Coverage enabled" FORCE)
else()
    set(AXON_ENABLE_COVERAGE OFF CACHE BOOL "Coverage enabled" FORCE)
endif()

if(AXON_ENABLE_COVERAGE)
    set(AXON_MCAP_ENABLE_COVERAGE ON CACHE BOOL "Enable coverage for axon_mcap" FORCE)
else()
    set(AXON_MCAP_ENABLE_COVERAGE OFF CACHE BOOL "Enable coverage for axon_mcap" FORCE)
endif()

add_subdirectory(${CPP_MCAP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)

# =============================================================================
# ROS 2 Setup
# =============================================================================
find_package(ament_cmake REQUIRED)
set(${PROJECT_NAME}_MEMBER_OF_GROUPS "rosidl_interface_packages" CACHE INTERNAL "" FORCE)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

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

# =============================================================================
# Auto-discover recorder library sources
# =============================================================================
file(GLOB_RECURSE RECORDER_LIB_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)
list(FILTER RECORDER_LIB_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# =============================================================================
# Build Library
# =============================================================================
add_library(axon_recorder_lib SHARED ${RECORDER_LIB_SOURCES})
target_compile_definitions(axon_recorder_lib PRIVATE
    AXON_ROS2
    BOOST_LOG_DYN_LINK
)
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
target_include_directories(axon_recorder_lib PRIVATE ${Boost_INCLUDE_DIRS})

if(AXON_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(axon_recorder_lib PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(axon_recorder_lib PRIVATE --coverage)
    endif()
endif()

rosidl_get_typesupport_target(cpp_typesupport_target_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(axon_recorder_lib "${cpp_typesupport_target_lib}")

if(ZSTD_FOUND)
    target_link_libraries(axon_recorder_lib ${ZSTD_LIBRARIES})
endif()
if(LZ4_FOUND)
    target_link_libraries(axon_recorder_lib ${LZ4_LIBRARIES})
endif()

# =============================================================================
# Executable
# =============================================================================
add_executable(axon_recorder_node src/main.cpp)
if(AXON_ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(axon_recorder_node PRIVATE --coverage -fprofile-arcs -ftest-coverage)
        target_link_options(axon_recorder_node PRIVATE --coverage)
    endif()
endif()
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
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(axon_recorder_node "${cpp_typesupport_target}")

# =============================================================================
# Install
# =============================================================================
install(TARGETS axon_recorder_lib axon_recorder_node axon_mcap
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY srv/ DESTINATION share/${PROJECT_NAME}/srv FILES_MATCHING PATTERN "*.srv")

# =============================================================================
# Tests
# =============================================================================
set(AXON_ROS2 TRUE)
add_subdirectory(test)

ament_package()

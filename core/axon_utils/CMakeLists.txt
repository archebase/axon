cmake_minimum_required(VERSION 3.10)
project(axon_utils LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem thread log)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_package(OpenSSL REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

# Source files - Configuration Parser
set(CONFIG_PARSER_SRC
  config_parser.cpp
)

# Source files - HTTP Client
set(HTTP_CLIENT_SRC
  http_client.cpp
)

# Source files - Common Types
set(COMMON_TYPES_SRC
  common_types.cpp
)

# Create the axon_utils library
add_library(axon_utils STATIC
  ${CONFIG_PARSER_SRC}
  ${HTTP_CLIENT_SRC}
  ${COMMON_TYPES_SRC}
)

# Link libraries
target_link_libraries(axon_utils
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
)

# Installation
install(TARGETS axon_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install headers
file(GLOB_RECURSE AXON_UTILS_HEADERS "*.hpp")
install(FILES ${AXON_UTILS_HEADERS} DESTINATION include)

# Enable testing
enable_testing()

# Add test subdirectory
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt")
  add_subdirectory(test)
endif()

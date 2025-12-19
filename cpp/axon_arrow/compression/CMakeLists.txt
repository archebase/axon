# Compression module CMake configuration
# This module provides image compression for robotics data

# Find pkg-config for compression library detection
find_package(PkgConfig QUIET)

# =============================================================================
# Verify Arrow compression codecs (LZ4, ZSTD)
# Note: Arrow's CMake may warn about missing lz4Config.cmake/zstdConfig.cmake
# These warnings are harmless - Arrow falls back to pkg-config successfully
# =============================================================================
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LZ4 QUIET liblz4)
    pkg_check_modules(ZSTD QUIET libzstd)
endif()

# Print compression status summary (appears after Arrow's warnings)
message(STATUS "")
message(STATUS "=== Axon Compression Status ===")
if(LZ4_FOUND)
    message(STATUS "  LZ4:  ✓ Available (via pkg-config)")
else()
    message(STATUS "  LZ4:  ✗ Not found - install liblz4-dev")
endif()
if(ZSTD_FOUND)
    message(STATUS "  ZSTD: ✓ Available (via pkg-config)")
else()
    message(STATUS "  ZSTD: ✗ Not found - install libzstd-dev")
endif()

# Find optional JPEG libraries
if(PKG_CONFIG_FOUND)
    pkg_check_modules(TURBOJPEG QUIET libturbojpeg)
endif()

if(NOT TURBOJPEG_FOUND)
    find_package(JPEG QUIET)
endif()

# Define compression library sources
set(COMPRESSION_SOURCES
    image_compressor.cpp
)

set(COMPRESSION_HEADERS
    image_compressor.hpp
)

# Create compression library
add_library(axon_compression ${COMPRESSION_SOURCES})

target_include_directories(axon_compression
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/axon/compression>
)

# Link Arrow for LZ4/ZSTD codecs
target_link_libraries(axon_compression
    PUBLIC
        Arrow::arrow_shared
)

# Link JPEG library if available
if(TURBOJPEG_FOUND)
    message(STATUS "Found libturbojpeg - enabling high-performance JPEG compression")
    target_compile_definitions(axon_compression PRIVATE HAS_TURBOJPEG=1)
    target_include_directories(axon_compression PRIVATE ${TURBOJPEG_INCLUDE_DIRS})
    target_link_libraries(axon_compression PRIVATE ${TURBOJPEG_LIBRARIES})
elseif(JPEG_FOUND)
    message(STATUS "Found libjpeg - enabling JPEG compression")
    target_compile_definitions(axon_compression PRIVATE HAS_JPEGLIB=1)
    target_include_directories(axon_compression PRIVATE ${JPEG_INCLUDE_DIRS})
    target_link_libraries(axon_compression PRIVATE ${JPEG_LIBRARIES})
else()
    message(WARNING "No JPEG library found - JPEG compression will be disabled")
    message(STATUS "  To enable JPEG: brew install jpeg-turbo (macOS) or apt install libturbojpeg0-dev (Ubuntu)")
endif()

# Set library properties
set_target_properties(axon_compression PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    PUBLIC_HEADER "${COMPRESSION_HEADERS}"
)

# Install targets
# Use the same export set as axon_arrow so both are exported together
install(TARGETS axon_compression
    EXPORT axon_arrow-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/axon/compression
)

# Export for parent CMake
set(AXON_COMPRESSION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

cmake_minimum_required(VERSION 3.12)

project(axon_mock CXX)

# Build mock middleware plugin for E2E testing
# Note: No external ABI header dependency - plugin defines its own ABI structures
add_library(axon_mock MODULE
    mock_middleware.cpp
)

target_compile_features(axon_mock PRIVATE cxx_std_17)

# Set output properties
set_target_properties(axon_mock PROPERTIES
    PREFIX ""
    OUTPUT_NAME "axon_mock"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Installation (optional)
install(TARGETS axon_mock
    LIBRARY DESTINATION lib/middlewares
)


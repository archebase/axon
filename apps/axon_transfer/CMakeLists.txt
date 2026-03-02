cmake_minimum_required(VERSION 3.16)
project(axon_transfer LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

if(NOT TARGET axon_uploader)
  message(FATAL_ERROR "axon_transfer requires target axon_uploader. Configure with -DAXON_BUILD_UPLOADER=ON")
endif()

set(AXON_TRANSFER_CORE_SOURCES
    src/transfer_config.cpp
    src/ws_client.cpp
    src/file_scanner.cpp
    src/upload_coordinator.cpp
    src/transfer_daemon.cpp
)

add_library(axon_transfer_core ${AXON_TRANSFER_CORE_SOURCES})

target_include_directories(
    axon_transfer_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(
    axon_transfer_core
    PUBLIC
        axon_uploader
        axon_logging
        yaml-cpp
        OpenSSL::Crypto
        OpenSSL::SSL
        Boost::system
        Boost::thread
        Boost::filesystem
        pthread
)

if(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(axon_transfer_core PUBLIC nlohmann_json::nlohmann_json)
elseif(DEFINED nlohmann_json_SOURCE_DIR)
  target_include_directories(
      axon_transfer_core
      PUBLIC ${nlohmann_json_SOURCE_DIR}/single_include
  )
endif()

target_compile_features(axon_transfer_core PUBLIC cxx_std_17)

add_executable(axon_transfer src/main.cpp)
target_link_libraries(axon_transfer PRIVATE axon_transfer_core)

install(TARGETS axon_transfer RUNTIME DESTINATION bin)

if(AXON_BUILD_TESTS)
  add_subdirectory(test)
endif()

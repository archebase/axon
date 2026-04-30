# =============================================================================
# axon_memory tests
# =============================================================================

cmake_minimum_required(VERSION 3.12)

if(DEFINED AXON_CMAKE_MODULES_DIR)
    list(APPEND CMAKE_MODULE_PATH "${AXON_CMAKE_MODULES_DIR}")
elseif(DEFINED AXON_REPO_ROOT)
    list(APPEND CMAKE_MODULE_PATH "${AXON_REPO_ROOT}/cmake/Modules")
else()
    set(_AXON_CMAKE_MODULES_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake/Modules"
    )
    if(EXISTS "${_AXON_CMAKE_MODULES_DIR}/AxonStdLib.cmake")
        list(APPEND CMAKE_MODULE_PATH "${_AXON_CMAKE_MODULES_DIR}")
    endif()
endif()

include(AxonStdLib REQUIRED)
include(AxonCoverage OPTIONAL)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    add_executable(${TEST_NAME} ${TEST_SOURCE})

    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

    target_link_libraries(
        ${TEST_NAME}
        axon_memory
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
    )

    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

    if(COMMAND axon_add_coverage)
        axon_add_coverage(${TEST_NAME})
    endif()
endforeach()

cmake_minimum_required(VERSION 3.15)

# ============================================================================
# Axon Recorder Tests
# ============================================================================
# Fail-fast: If dependencies are missing, build fails immediately.
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect ROS build mode (must be explicitly set)
if(DEFINED AXON_ROS1)
    set(ROS_INTEGRATED_BUILD TRUE)
    set(AXON_ROS_VERSION 1)
    message(STATUS "Building tests with ROS 1 integration")
elseif(DEFINED AXON_ROS2)
    set(ROS_INTEGRATED_BUILD TRUE)
    set(AXON_ROS_VERSION 2)
    message(STATUS "Building tests with ROS 2 integration")
else()
    message(STATUS "Building standalone tests (no ROS)")
    project(axon_recorder_tests)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(GTest REQUIRED)
include(GoogleTest)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
find_package(yaml-cpp REQUIRED)

# Path to axon_logging (fixed location)
set(CPP_LOGGING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_logging)
add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)

enable_testing()

# Default test timeout
set(AXON_TEST_TIMEOUT 60 CACHE STRING "Default test timeout in seconds")

# ============================================================================
# Test Macros
# ============================================================================
macro(axon_add_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    add_executable(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(${TEST_NAME} GTest::gtest GTest::gtest_main Threads::Threads ${TEST_LIBS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT})
endmacro()

macro(axon_add_ros1_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    add_executable(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS1)
    target_include_directories(${TEST_NAME} PRIVATE ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} ${TEST_LIBS})
    add_dependencies(${TEST_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT} LABELS "integration")
endmacro()

macro(axon_add_ros2_test)
    cmake_parse_arguments(TEST "" "NAME" "SOURCES;LIBS" ${ARGN})
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCES})
    target_compile_definitions(${TEST_NAME} PRIVATE AXON_ROS2)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
    target_link_libraries(${TEST_NAME} ${TEST_LIBS})
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(${TEST_NAME} "${cpp_typesupport_target}")
    set_tests_properties(${TEST_NAME} PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH};AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
        LABELS "integration"
    )
endmacro()

macro(axon_discover_unit_tests)
    cmake_parse_arguments(DISCOVER "" "DIR;LABEL" "LIBS;EXCLUDE" ${ARGN})
    file(GLOB _DISCOVER_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${DISCOVER_DIR}/test_*.cpp")
    if(DISCOVER_EXCLUDE)
        foreach(_EXCLUDE_PATTERN ${DISCOVER_EXCLUDE})
            list(FILTER _DISCOVER_TEST_SOURCES EXCLUDE REGEX "${_EXCLUDE_PATTERN}")
        endforeach()
    endif()
    foreach(_TEST_SOURCE ${_DISCOVER_TEST_SOURCES})
        get_filename_component(_TEST_NAME ${_TEST_SOURCE} NAME_WE)
        add_executable(${_TEST_NAME} ${_TEST_SOURCE})
        target_compile_definitions(${_TEST_NAME} PRIVATE BOOST_LOG_DYN_LINK)
        target_include_directories(${_TEST_NAME} PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../src
            ${CMAKE_CURRENT_SOURCE_DIR}/${DISCOVER_DIR}
        )
        target_link_libraries(${_TEST_NAME} GTest::gtest GTest::gtest_main Threads::Threads ${DISCOVER_LIBS})
        gtest_discover_tests(${_TEST_NAME} PROPERTIES LABELS "${DISCOVER_LABEL}" TIMEOUT ${AXON_TEST_TIMEOUT})
    endforeach()
    list(LENGTH _DISCOVER_TEST_SOURCES _DISCOVER_COUNT)
    message(STATUS "Auto-discovered ${_DISCOVER_COUNT} tests in ${DISCOVER_DIR}/")
endmacro()

# ============================================================================
# Test Support Libraries
# ============================================================================
add_library(axon_recorder_test_support STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_service_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/logging/axon_ros_sink.cpp
)
target_compile_definitions(axon_recorder_test_support PRIVATE BOOST_LOG_DYN_LINK)
target_include_directories(axon_recorder_test_support PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(axon_recorder_test_support
    axon_logging yaml-cpp OpenSSL::SSL OpenSSL::Crypto Threads::Threads ${Boost_LIBRARIES}
)

# MCAP support library (ROS builds only)
if(ROS_INTEGRATED_BUILD)
    add_library(axon_recorder_test_support_mcap STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
    )
    target_compile_definitions(axon_recorder_test_support_mcap PRIVATE BOOST_LOG_DYN_LINK)
    target_include_directories(axon_recorder_test_support_mcap PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
    )
    target_link_libraries(axon_recorder_test_support_mcap
        axon_mcap axon_logging nlohmann_json::nlohmann_json OpenSSL::SSL OpenSSL::Crypto ${Boost_LIBRARIES}
    )
endif()

# ============================================================================
# Unit Tests (Auto-discovered)
# ============================================================================
axon_discover_unit_tests(
    DIR unit
    LIBS axon_recorder_test_support axon_logging
    LABEL unit
)

# Integration Tests
axon_add_test(NAME test_recording_workflow
    SOURCES integration/test_recording_workflow.cpp
    LIBS axon_recorder_test_support axon_logging
)
set_tests_properties(test_recording_workflow PROPERTIES LABELS "integration")

# Regression Tests
axon_add_test(NAME test_error_recovery
    SOURCES regression/test_error_recovery.cpp
    LIBS axon_recorder_test_support axon_logging
)

# Tests Requiring MCAP
if(ROS_INTEGRATED_BUILD)
    axon_add_test(NAME test_recording_session
        SOURCES integration/test_recording_session.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_recording_session PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)

    axon_add_test(NAME test_metadata_injector
        SOURCES integration/test_metadata_injector.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_metadata_injector PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)

    axon_add_test(NAME test_metadata_injection
        SOURCES integration/test_metadata_injection.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_metadata_injection PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)
    set_tests_properties(test_metadata_injection PROPERTIES LABELS "integration")
endif()

# Tests Requiring ROS
if(ROS_INTEGRATED_BUILD)
    if(DEFINED AXON_ROS1)
        find_package(rostest REQUIRED)
        set(ROS1_TEST_LIBS GTest::gtest GTest::gtest_main Threads::Threads ${catkin_LIBRARIES} ${Boost_LIBRARIES})

        axon_add_ros1_test(NAME test_ros_introspection_ros1
            SOURCES integration/test_ros_introspection.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/ros_introspection.cpp
            LIBS ${ROS1_TEST_LIBS}
        )

        axon_add_ros1_test(NAME test_message_factory_ros1
            SOURCES integration/test_message_factory.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/message_factory.cpp
            LIBS ${ROS1_TEST_LIBS}
        )

    elseif(DEFINED AXON_ROS2)
        find_package(ament_cmake_gtest REQUIRED)
        set(ROS2_TEST_LIBS GTest::gtest GTest::gtest_main Threads::Threads rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS})

        axon_add_ros2_test(NAME test_ros_introspection_ros2
            SOURCES integration/test_ros_introspection.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/ros_introspection.cpp
            LIBS ${ROS2_TEST_LIBS}
        )

        axon_add_ros2_test(NAME test_message_factory_ros2
            SOURCES integration/test_message_factory.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../src/message_factory.cpp
            LIBS ${ROS2_TEST_LIBS}
        )
    endif()
endif()

# ROS Integration Tests
if(ROS_INTEGRATED_BUILD)
    set(ROS_INT_BASE_LIBS axon_recorder_lib Threads::Threads ${Boost_LIBRARIES})

    if(DEFINED AXON_ROS1)
        list(APPEND ROS_INT_BASE_LIBS ${catkin_LIBRARIES})
        set(ROS1_INT_LIBS ${ROS_INT_BASE_LIBS} GTest::gtest GTest::gtest_main)

        axon_add_ros1_test(NAME test_service_adapter_ros1 SOURCES integration/test_service_adapter.cpp LIBS ${ROS1_INT_LIBS})
        axon_add_ros1_test(NAME test_ros_interface_ros1 SOURCES integration/test_ros_interface.cpp LIBS ${ROS1_INT_LIBS})
        axon_add_ros1_test(NAME test_recorder_node_lifecycle_ros1 SOURCES integration/test_recorder_node_lifecycle.cpp LIBS ${ROS1_INT_LIBS})
        axon_add_ros1_test(NAME test_recorder_node_recording_ros1 SOURCES integration/test_recorder_node_recording.cpp LIBS ${ROS1_INT_LIBS})
        axon_add_ros1_test(NAME test_recording_services_ros1 SOURCES integration/test_recording_services.cpp LIBS ${ROS1_INT_LIBS})

    elseif(DEFINED AXON_ROS2)
        list(APPEND ROS_INT_BASE_LIBS rclcpp::rclcpp ${std_msgs_TARGETS})
        set(ROS2_INT_LIBS ${ROS_INT_BASE_LIBS} GTest::gtest GTest::gtest_main)

        axon_add_ros2_test(NAME test_service_adapter_ros2 SOURCES integration/test_service_adapter.cpp LIBS ${ROS2_INT_LIBS})
        axon_add_ros2_test(NAME test_ros_interface_ros2 SOURCES integration/test_ros_interface.cpp LIBS ${ROS2_INT_LIBS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})
        axon_add_ros2_test(NAME test_recorder_node_lifecycle_ros2 SOURCES integration/test_recorder_node_lifecycle.cpp LIBS ${ROS2_INT_LIBS})
        axon_add_ros2_test(NAME test_recorder_node_recording_ros2 SOURCES integration/test_recorder_node_recording.cpp LIBS ${ROS2_INT_LIBS})
        axon_add_ros2_test(NAME test_recording_services_ros2 SOURCES integration/test_recording_services.cpp LIBS ${ROS2_INT_LIBS})
    endif()
endif()

# Stress Tests
if(ROS_INTEGRATED_BUILD)
    axon_add_test(NAME test_recording_session_stress
        SOURCES stress/test_recording_session_stress.cpp
        LIBS axon_recorder_test_support_mcap axon_logging
    )
    target_include_directories(test_recording_session_stress PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/../generated)
    set_tests_properties(test_recording_session_stress PROPERTIES LABELS "stress" TIMEOUT 300)
endif()

# Edge Uploader Integration Tests
if(AXON_HAS_UPLOADER AND ROS_INTEGRATED_BUILD)
    axon_add_test(NAME test_recorder_node_uploader
        SOURCES integration/test_recorder_node_uploader.cpp
        LIBS axon_recorder_test_support_mcap axon_uploader axon_logging
    )
    target_compile_definitions(test_recorder_node_uploader PRIVATE AXON_HAS_UPLOADER)
    target_include_directories(test_recorder_node_uploader PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_uploader
    )
    set_tests_properties(test_recorder_node_uploader PROPERTIES LABELS "integration" TIMEOUT ${AXON_TEST_TIMEOUT})
endif()

# Performance Tests
add_subdirectory(perf EXCLUDE_FROM_ALL)

# Run All Tests Target
add_custom_target(run_recorder_tests COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure COMMENT "Running all recorder tests")

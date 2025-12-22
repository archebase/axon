cmake_minimum_required(VERSION 3.15)

# ============================================================================
# Axon Recorder Tests
# ============================================================================
# Supports two modes:
# 1. Standalone build (cmake ..) - Unit tests only
# 2. ROS-integrated build (via main CMakeLists.txt) - Full tests with services
# ============================================================================

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Code Coverage Support (inherited from parent or standalone)
# ============================================================================
if(ENABLE_COVERAGE)
    message(STATUS "Test targets built with coverage instrumentation")
endif()

# Detect if we're being built as part of the ROS package
if(DEFINED AXON_ROS1 OR DEFINED AXON_ROS2)
    set(ROS_INTEGRATED_BUILD TRUE)
    message(STATUS "Building tests with ROS integration")
else()
    set(ROS_INTEGRATED_BUILD FALSE)
    message(STATUS "Building standalone tests (no ROS)")
    project(axon_recorder_tests)
endif()

# Find dependencies
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system log log_setup thread filesystem)
find_package(yaml-cpp REQUIRED)

# Path to axon_logging (needed for tests that compile source files directly)
if(NOT DEFINED CPP_LOGGING_DIR)
    set(CPP_LOGGING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp/axon_logging)
endif()
include_directories(${CPP_LOGGING_DIR})

enable_testing()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${GTEST_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# Build axon_logging for tests (unless already available from parent)
if(NOT TARGET axon_logging)
    add_subdirectory(${CPP_LOGGING_DIR} ${CMAKE_CURRENT_BINARY_DIR}/axon_logging)
endif()

# ============================================================================
# Unit Tests (Both standalone and ROS-integrated)
# ============================================================================

# Test TaskConfig and TaskConfigCache
add_executable(test_task_config
    unit/test_task_config.cpp
)
target_link_libraries(test_task_config
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME TaskConfigTest COMMAND test_task_config)

# Test StateManager
add_executable(test_state_machine
    unit/test_state_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
)
target_link_libraries(test_state_machine
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME StateMachineTest COMMAND test_state_machine)

# Test StateTransactionGuard
add_executable(test_state_transaction_guard
    unit/test_state_transaction_guard.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
)
target_link_libraries(test_state_transaction_guard
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME StateTransactionGuardTest COMMAND test_state_transaction_guard)

# Test HttpCallbackClient (requires Boost and OpenSSL for SSL support)
if(Boost_FOUND)
    add_executable(test_http_callback_client
        unit/test_http_callback_client.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    )
    target_compile_definitions(test_http_callback_client PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_http_callback_client
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
        axon_logging
        ${Boost_LIBRARIES}
    )
    add_test(NAME HttpCallbackClientTest COMMAND test_http_callback_client)

    # Test Recording Workflow (mock node)
    add_executable(test_recording_workflow
        integration/test_recording_workflow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    )
    target_compile_definitions(test_recording_workflow PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_recording_workflow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
        axon_logging
        ${Boost_LIBRARIES}
    )
    add_test(NAME RecordingWorkflowTest COMMAND test_recording_workflow)

    # Test RecordingServiceImpl with MockRecorderContext
    add_executable(test_recording_service_impl
        unit/test_recording_service_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_service_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    )
    target_include_directories(test_recording_service_impl PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/unit
    )
    target_compile_definitions(test_recording_service_impl PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_recording_service_impl
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
        axon_logging
        ${Boost_LIBRARIES}
    )
    add_test(NAME RecordingServiceImplTest COMMAND test_recording_service_impl)

    # Test IRecorderContext interface and MockRecorderContext
    add_executable(test_recorder_context_interface
        unit/test_recorder_context_interface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    )
    target_include_directories(test_recorder_context_interface PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/unit
    )
    target_compile_definitions(test_recorder_context_interface PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_recorder_context_interface
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
        axon_logging
        ${Boost_LIBRARIES}
    )
    add_test(NAME RecorderContextInterfaceTest COMMAND test_recorder_context_interface)
endif()

# Test TopicManager (requires ros_interface mock)
add_executable(test_topic_manager
    unit/test_topic_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
)
target_compile_definitions(test_topic_manager PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_topic_manager
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    axon_logging
    ${Boost_LIBRARIES}
)
add_test(NAME TopicManagerTest COMMAND test_topic_manager)

# Test WorkerThreadPool
add_executable(test_worker_thread_pool
    unit/test_worker_thread_pool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
)
target_compile_definitions(test_worker_thread_pool PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_worker_thread_pool
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    axon_logging
    ${Boost_LIBRARIES}
)
add_test(NAME WorkerThreadPoolTest COMMAND test_worker_thread_pool)

# Test ConfigParser (YAML parsing and validation)
add_executable(test_config_parser
    unit/test_config_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config_parser.cpp
)
target_compile_definitions(test_config_parser PRIVATE BOOST_LOG_DYN_LINK)
target_link_libraries(test_config_parser
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
    axon_logging
    yaml-cpp
    ${Boost_LIBRARIES}
)
add_test(NAME ConfigParserTest COMMAND test_config_parser)

# Test SPSCQueue (lock-free queue)
add_executable(test_spsc_queue
    unit/test_spsc_queue.cpp
)
target_link_libraries(test_spsc_queue
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME SPSCQueueTest COMMAND test_spsc_queue)

# ============================================================================
# Tests requiring ROS message types (only when built with ROS)
# ============================================================================

# Note: test_message_factory is disabled by default due to long compilation times
# from heavy ROS message header includes. Enable with -DBUILD_MESSAGE_FACTORY_TEST=ON
option(BUILD_MESSAGE_FACTORY_TEST "Build MessageFactory test (slow compilation)" OFF)

if(ROS_INTEGRATED_BUILD AND BUILD_MESSAGE_FACTORY_TEST)
    # Test MessageFactory (requires ROS message types)
    add_executable(test_message_factory
        unit/test_message_factory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/message_factory.cpp
    )
    if(DEFINED AXON_ROS1)
        target_compile_definitions(test_message_factory PRIVATE AXON_ROS1)
        target_include_directories(test_message_factory PRIVATE
            ${catkin_INCLUDE_DIRS}
        )
        target_link_libraries(test_message_factory
            GTest::gtest
            GTest::gtest_main
            Threads::Threads
            ${catkin_LIBRARIES}
        )
    elseif(DEFINED AXON_ROS2)
        target_compile_definitions(test_message_factory PRIVATE AXON_ROS2)
        target_link_libraries(test_message_factory
            GTest::gtest
            GTest::gtest_main
            Threads::Threads
            rclcpp::rclcpp
            ${std_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
        )
    endif()
    add_test(NAME MessageFactoryTest COMMAND test_message_factory)
endif()

# ============================================================================
# Tests requiring axon_mcap (only when built as part of ROS package)
# ============================================================================

if(ROS_INTEGRATED_BUILD)
    # Test RecordingSession (requires axon_mcap)
    add_executable(test_recording_session
        unit/test_recording_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
    )
    target_include_directories(test_recording_session PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
    )
    target_compile_definitions(test_recording_session PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_recording_session
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        axon_mcap
        axon_logging
        nlohmann_json::nlohmann_json
        OpenSSL::SSL
        OpenSSL::Crypto
        ${Boost_LIBRARIES}
    )
    add_test(NAME RecordingSessionTest COMMAND test_recording_session)

    # Test MetadataInjector (requires axon_mcap, nlohmann_json, OpenSSL)
    add_executable(test_metadata_injector
        unit/test_metadata_injector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
    )
    target_include_directories(test_metadata_injector PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
    )
    target_compile_definitions(test_metadata_injector PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_metadata_injector
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        axon_mcap
        axon_logging
        nlohmann_json::nlohmann_json
        OpenSSL::SSL
        OpenSSL::Crypto
        ${Boost_LIBRARIES}
    )
    add_test(NAME MetadataInjectorTest COMMAND test_metadata_injector)

    # Integration test: Metadata injection end-to-end
    add_executable(test_metadata_injection
        integration/test_metadata_injection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/metadata_injector.cpp
    )
    target_include_directories(test_metadata_injection PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/../generated
    )
    target_compile_definitions(test_metadata_injection PRIVATE BOOST_LOG_DYN_LINK)
    target_link_libraries(test_metadata_injection
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        axon_mcap
        axon_logging
        nlohmann_json::nlohmann_json
        OpenSSL::SSL
        OpenSSL::Crypto
        ${Boost_LIBRARIES}
    )
    add_test(NAME MetadataInjectionIntegrationTest COMMAND test_metadata_injection)
endif()

# ============================================================================
# ROS-Integrated Tests (Only when built with ROS)
# ============================================================================

if(ROS_INTEGRATED_BUILD AND Boost_FOUND)
    message(STATUS "Adding ROS-integrated tests")
    
    if(DEFINED AXON_ROS1)
        # ROS 1 integrated tests using catkin
        find_package(rostest QUIET)
        
        if(rostest_FOUND)
            # Test ServiceAdapter with ROS 1
            # Link against axon_recorder_lib instead of compiling sources directly
            add_executable(test_service_adapter_ros1
                integration/test_service_adapter.cpp
            )
            target_compile_definitions(test_service_adapter_ros1 PRIVATE AXON_ROS1)
            target_include_directories(test_service_adapter_ros1 PRIVATE
                ${catkin_INCLUDE_DIRS}
            )
            target_link_libraries(test_service_adapter_ros1
                axon_recorder_lib
                GTest::gtest
                GTest::gtest_main
                Threads::Threads
                ${catkin_LIBRARIES}
                ${Boost_LIBRARIES}
            )
            add_dependencies(test_service_adapter_ros1 ${${PROJECT_NAME}_EXPORTED_TARGETS})
            add_test(NAME ServiceAdapterROS1Test COMMAND test_service_adapter_ros1)
        endif()
        
    elseif(DEFINED AXON_ROS2)
        # ROS 2 integrated tests using ament
        find_package(ament_cmake_gtest QUIET)
        
        if(ament_cmake_gtest_FOUND)
            # Test ServiceAdapter with ROS 2
            # Link against axon_recorder_lib instead of compiling sources directly
            # This ensures all RecorderNode methods are available
            ament_add_gtest(test_service_adapter_ros2
                integration/test_service_adapter.cpp
            )
            target_compile_definitions(test_service_adapter_ros2 PRIVATE AXON_ROS2)
            target_include_directories(test_service_adapter_ros2 PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/../src
            )
            ament_target_dependencies(test_service_adapter_ros2
                rclcpp
                std_msgs
            )
            target_link_libraries(test_service_adapter_ros2
                axon_recorder_lib
                Threads::Threads
                ${Boost_LIBRARIES}
            )
            # Link to generated service types
            rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
            target_link_libraries(test_service_adapter_ros2 "${cpp_typesupport_target}")
        endif()
    endif()
endif()

# ============================================================================
# Performance Tests
# ============================================================================

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/perf/CMakeLists.txt)
    add_subdirectory(perf)
endif()

# ============================================================================
# Custom Target: Run All Tests
# ============================================================================
add_custom_target(run_recorder_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMENT "Running all recorder tests"
)

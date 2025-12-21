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
find_package(Boost QUIET COMPONENTS system)
if(NOT Boost_FOUND)
    find_package(Boost QUIET)
endif()

enable_testing()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${GTEST_INCLUDE_DIRS}
)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
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
    target_link_libraries(test_http_callback_client
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
    )
    if(Boost_SYSTEM_FOUND)
        target_link_libraries(test_http_callback_client ${Boost_LIBRARIES})
    endif()
    add_test(NAME HttpCallbackClientTest COMMAND test_http_callback_client)

    # Test Recording Workflow (mock node)
    add_executable(test_recording_workflow
        integration/test_recording_workflow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    )
    target_link_libraries(test_recording_workflow
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
    )
    if(Boost_SYSTEM_FOUND)
        target_link_libraries(test_recording_workflow ${Boost_LIBRARIES})
    endif()
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
    target_link_libraries(test_recording_service_impl
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
    )
    if(Boost_SYSTEM_FOUND)
        target_link_libraries(test_recording_service_impl ${Boost_LIBRARIES})
    endif()
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
    target_link_libraries(test_recorder_context_interface
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
    )
    if(Boost_SYSTEM_FOUND)
        target_link_libraries(test_recorder_context_interface ${Boost_LIBRARIES})
    endif()
    add_test(NAME RecorderContextInterfaceTest COMMAND test_recorder_context_interface)
endif()

# Test TopicManager (requires ros_interface mock)
add_executable(test_topic_manager
    unit/test_topic_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
)
target_link_libraries(test_topic_manager
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME TopicManagerTest COMMAND test_topic_manager)

# Test WorkerThreadPool
add_executable(test_worker_thread_pool
    unit/test_worker_thread_pool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
)
target_link_libraries(test_worker_thread_pool
    GTest::gtest
    GTest::gtest_main
    Threads::Threads
)
add_test(NAME WorkerThreadPoolTest COMMAND test_worker_thread_pool)

# ============================================================================
# Tests requiring axon_mcap (only when built as part of ROS package)
# ============================================================================

if(ROS_INTEGRATED_BUILD)
    # Test RecordingSession (requires axon_mcap)
    add_executable(test_recording_session
        unit/test_recording_session.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_session.cpp
    )
    target_link_libraries(test_recording_session
        GTest::gtest
        GTest::gtest_main
        Threads::Threads
        axon_mcap
    )
    add_test(NAME RecordingSessionTest COMMAND test_recording_session)
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

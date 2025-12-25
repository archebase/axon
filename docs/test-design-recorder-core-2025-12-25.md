# Test Design Document: Axon Recorder Core Components

**Document Version:** 1.6  
**Date:** 2025-12-25  
**Author:** Axon Robotics Team  
**Reviewers:** Staff/Principal Engineers  

## Executive Summary

This document provides a comprehensive test design for the `axon_recorder` package, focusing on three critical components:

1. **`recorder_node.cpp`** - The main orchestrator (~865 lines)
2. **`recording_session.cpp`** - MCAP session lifecycle management (~255 lines)
3. **`recording_service_impl.cpp`** - ROS service implementation (~430 lines)

### Current Test Status

The test suite currently has **590 tests** across **23 test files**:
- **17 unit test files** (467 tests) - Fast, no ROS runtime required
- **6 integration test files** (100 tests) - Require ROS environment
- **1 regression test file** (23 tests) - Error recovery scenarios

The goal is to achieve **>90% line coverage** and **>85% branch coverage** while ensuring robust testing of:
- State machine transitions
- Concurrency and thread safety
- Error handling and recovery
- ROS1/ROS2 dual compatibility
- Edge cases and boundary conditions

---

## Table of Contents

1. [Component Analysis](#1-component-analysis)
2. [Current Test Coverage Assessment](#2-current-test-coverage-assessment)
3. [Test Strategy](#3-test-strategy)
4. [Detailed Test Plans](#4-detailed-test-plans)
5. [Test Infrastructure Improvements](#5-test-infrastructure-improvements)
6. [Implementation Priority](#6-implementation-priority)
7. [Appendices](#appendices)

---

## 1. Component Analysis

### 1.1 RecorderNode (`recorder_node.cpp`)

**Responsibilities:**
- ROS interface lifecycle management
- Worker thread pool orchestration
- MCAP writer initialization and management
- Topic subscription and message routing
- Edge uploader integration (conditional)
- HTTP callback client coordination
- Statistics collection and reporting

**Key Code Paths:**

| Function | Lines | Complexity | Current Coverage |
|----------|-------|------------|------------------|
| `initialize()` | 83-137 | High | Partial |
| `complete_initialization()` | 139-216 | High | Partial |
| `setup_topic_recording()` | 474-547 | Medium | Low |
| `start_recording()` | 561-590 | Medium | Good |
| `stop_recording()` | 655-754 | High | Partial |
| `cancel_recording()` | 608-653 | Medium | Partial |
| `shutdown()` | 225-290 | High | Partial |
| `configure_from_task_config()` | 756-787 | Low | Good |
| `write_stats_file()` | 816-861 | Low | None |

**Critical Branches:**
- Conditional compilation: `#ifdef AXON_HAS_UPLOADER`
- ROS version detection: `#if defined(AXON_ROS1)` vs `#elif defined(AXON_ROS2)`
- Topic type detection (Image, IMU, etc.)
- Error handling in initialization

### 1.2 RecordingSession (`recording_session.cpp`)

**Responsibilities:**
- MCAP file open/close lifecycle
- Schema and channel registration
- Thread-safe message writing
- Topic statistics tracking
- Metadata injection coordination
- Sidecar JSON generation

**Key Code Paths:**

| Function | Lines | Complexity | Current Coverage |
|----------|-------|------------|------------------|
| `open()` | 21-55 | Medium | Good |
| `close()` | 57-96 | High | Partial |
| `register_schema()` | 108-136 | Medium | Good |
| `register_channel()` | 138-168 | Medium | Good |
| `write()` | 170-182 | Low | Good |
| `update_topic_stats()` | 241-251 | Low | Partial |
| `get_stats()` | 196-209 | Low | Good |

**Thread Safety Concerns:**
- `registry_mutex_` protects schema/channel maps
- `topic_stats_mutex_` protects topic statistics
- `messages_written_` is atomic

### 1.3 RecordingServiceImpl (`recording_service_impl.cpp`)

**Responsibilities:**
- Service request validation
- State machine transition coordination
- Task config caching
- Recording lifecycle commands (start/stop/pause/resume/cancel/clear)
- Status reporting

**Key Code Paths:**

| Function | Lines | Complexity | Current Coverage |
|----------|-------|------------|------------------|
| `handle_cached_recording_config()` | 22-78 | Medium | Good |
| `handle_is_recording_ready()` | 84-131 | Low | Good |
| `handle_recording_control()` | 137-165 | Low | Good |
| `handle_start_command()` | 167-204 | Medium | Good |
| `handle_pause_command()` | 206-234 | Medium | Good |
| `handle_resume_command()` | 236-264 | Medium | Good |
| `handle_cancel_command()` | 266-294 | Medium | Good |
| `handle_finish_command()` | 296-324 | Medium | Good |
| `handle_clear_command()` | 326-348 | Low | Good |
| `handle_recording_status()` | 354-408 | Medium | Good |
| `validate_task_id()` | 414-427 | Low | Good |

---

## 2. Current Test Coverage Assessment

### 2.1 Existing Test Files

#### Unit Tests (`test/unit/`)

| Test File | Target Component | Test Count |
|-----------|------------------|------------|
| `test_config_parser.cpp` | ConfigParser | 66 |
| `test_http_callback_client.cpp` | HttpCallbackClient | 70 |
| `test_recording_service_impl.cpp` | RecordingServiceImpl | 51 |
| `test_metadata_injector.cpp` | MetadataInjector | 42 |
| `test_state_machine.cpp` | StateManager | 36 |
| `test_worker_thread_pool.cpp` | WorkerThreadPool | 31 |
| `test_recording_session.cpp` | RecordingSession | 24 |
| `test_topic_manager.cpp` | TopicManager | 24 |
| `test_spsc_queue.cpp` | SPSCQueue | 23 |
| `test_recorder_context_interface.cpp` | IRecorderContext | 19 |
| `test_ros_introspection.cpp` | ROS Introspection | 17 |
| `test_ros_sink.cpp` | AxonRosSink | 16 |
| `test_message_factory_standalone.cpp` | MessageFactory | 15 |
| `test_task_config.cpp` | TaskConfig | 14 |
| `test_message_factory.cpp` | MessageFactory (ROS) | 11 |
| `test_state_transaction_guard.cpp` | StateTransactionGuard | 8 |

**Mock Infrastructure:**
- `mock_recorder_context.hpp` - MockRecorderContext for testing RecordingServiceImpl
- `mock_ros_interface_logging.hpp` - Mock ROS logging interface

#### Integration Tests (`test/integration/`)

| Test File | Target Component | Test Count |
|-----------|------------------|------------|
| `test_ros_interface.cpp` | RosInterface | 23 |
| `test_recording_workflow.cpp` | Full recording workflow | 20 |
| `test_recorder_node_recording.cpp` | RecorderNode recording | 19 |
| `test_recorder_node_lifecycle.cpp` | RecorderNode lifecycle | 17 |
| `test_service_adapter.cpp` | ServiceAdapter | 15 |
| `test_metadata_injection.cpp` | Metadata injection | 6 |

**E2E Scripts:**
- `run_e2e_tests.sh` - Full E2E test runner
- `test_ros_services.sh` - ROS service interface tests

#### Regression Tests (`test/regression/`)

| Test File | Focus | Test Count |
|-----------|-------|------------|
| `test_error_recovery.cpp` | Error recovery scenarios | 23 |

#### Performance Tests (`test/perf/`)

| File | Purpose |
|------|---------|
| `perf_test_node.cpp` | Performance benchmarking node |
| `synthetic_publisher.cpp` | Test data publisher for load testing |
| `run_perf_test.sh` | Performance test runner script |
| `CMakeLists.txt` | Build configuration for perf tests |

### 2.2 Coverage Gaps Identified

> **Note:** With 590 tests, many code paths are well-covered. The gaps below focus on remaining uncovered paths and edge cases identified through coverage analysis.

#### Root Cause Analysis (Coverage < 50%)

The following files have been identified as having structural coverage challenges:

| File | Coverage | Root Cause | Resolution |
|------|----------|------------|------------|
| `register_common_messages.cpp` | ~0% → **Fixed** | ROS2 `init()` was missing call to `register_common_message_types()` | ✅ Added call in ROS2 `RosInterfaceImpl::init()` |
| `recorder_node.cpp` | ~31% | Main entry point; requires full ROS stack for integration tests | Integration tests cover critical paths |
| `http_callback_client.cpp` | ~34% | Network code requires HTTP server mocking or real endpoints | Consider adding HTTP mock server |
| `message_factory.cpp` | ~37% | Template-heavy code; MessageFactory not exercised without ROS init | Covered by `register_common_messages` fix |
| `ros_interface.cpp` | ~38% | ROS-dependent code paths; conditional ROS1/ROS2 compilation | Each distro covers its own paths |
| `metadata_injector.cpp` | ~38% | Complex metadata generation; many edge cases | Add targeted unit tests |
| `recording_service_impl.cpp` | ~40% | Service handlers well-tested; error paths less covered | Add error injection tests |

#### Coverage Collection Strategy

Coverage is collected from **multiple sources** and merged in Codecov:

| Workflow | Flag | Coverage Source |
|----------|------|-----------------|
| `unit-tests-ros.yml` | `ros1`, `ros2` | ROS unit tests (Humble/Noetic) |
| `unit-tests-cpp.yml` | `cpp` | C++ library tests |
| `integration-tests.yml` | `ros1`, `ros2` | Integration tests (Humble/Noetic) |
| `e2e-tests.yml` | `ros2` | E2E tests (Humble only) |

**Important:** ROS1-only code paths (e.g., `#if defined(AXON_ROS1)`) are only covered by `ros1` flag from Noetic builds. Similarly, ROS2-only paths require `ros2` flag.

#### RecorderNode Coverage Gaps

| Gap Category | Functions/Paths | Priority | Notes |
|--------------|-----------------|----------|-------|
| **Edge Uploader** | `#ifdef AXON_HAS_UPLOADER` paths | P1 | Conditional compilation - requires mock |
| **Stats File Writing** | `write_stats_file()` | P1 | Only tested in integration tests |
| **Schema Registration Failures** | `register_topic_schemas()` error paths | P2 | Error injection needed |
| **Worker Pool Error Paths** | `setup_topic_recording()` failure modes | P2 | Edge cases |

#### RecordingSession Coverage Gaps

| Gap Category | Functions/Paths | Priority | Notes |
|--------------|-----------------|----------|-------|
| **Metadata Injection** | `close()` with task config | P1 | Partially covered in `test_metadata_injection.cpp` |
| **Sidecar Generation** | `close()` → sidecar JSON | P1 | Needs validation tests |
| **Concurrent Write Stress** | Multiple threads writing | P2 | Currently manual testing |

#### RecordingServiceImpl Coverage Gaps

| Gap Category | Functions/Paths | Priority | Notes |
|--------------|-----------------|----------|-------|
| **State Transition Failures** | `transition_to()` returning false | P2 | Well-covered; edge cases remain |
| **Stress Testing** | High-frequency concurrent calls | P2 | Not in automated tests |

---

## 3. Test Strategy

### 3.1 Testing Pyramid

```
                    ╭─────────────────╮
                    │   E2E Tests     │  ← ROS service calls, full integration
                    │   (10 tests)    │
                ╭───┴─────────────────┴───╮
                │  Integration Tests      │  ← RecorderNode + ROS
                │  (30 tests)             │
            ╭───┴─────────────────────────┴───╮
            │        Unit Tests               │  ← Isolated component tests
            │        (100+ tests)             │
        ╭───┴─────────────────────────────────┴───╮
        │        Mock Infrastructure              │
        │   MockRecorderContext, MockRosInterface │
        ╰─────────────────────────────────────────╯
```

### 3.2 Test Categories

| Category | Focus | Framework | ROS Required |
|----------|-------|-----------|--------------|
| **Unit** | Isolated logic, mocked dependencies | GTest | No |
| **Component** | Single component with real deps | GTest | Partial |
| **Integration** | Multiple components, real ROS | GTest + ROS | Yes |
| **E2E** | Full system via service calls | Shell/Python | Yes |
| **Performance** | Throughput, latency, memory | Custom | Yes |
| **Stress** | Concurrency, resource limits | GTest | No |

### 3.3 ROS1/ROS2 Dual Testing Strategy

All tests must pass on both:
- **ROS1 Noetic** (Ubuntu 20.04, GCC 9)
- **ROS2 Humble** (Ubuntu 22.04, GCC 11)
- **ROS2 Jazzy** (Ubuntu 24.04, GCC 13)
- **ROS2 Rolling** (Latest)

Use conditional compilation guards:
```cpp
#if defined(AXON_ROS1)
  // ROS1-specific test setup
#elif defined(AXON_ROS2)
  // ROS2-specific test setup
#endif
```

### 3.4 Current Test Folder Structure

The test suite is organized as follows:

```
test/
├── CMakeLists.txt                     # Auto-discovery macros + test registration
│
├── unit/                              # Fast, isolated, no ROS runtime (17 files)
│   ├── mock_recorder_context.hpp          ← MockRecorderContext for service tests
│   ├── mock_ros_interface_logging.hpp     ← Mock ROS logging interface
│   ├── test_config_parser.cpp             ← ConfigParser (66 tests)
│   ├── test_http_callback_client.cpp      ← HttpCallbackClient (70 tests)
│   ├── test_message_factory.cpp           ← MessageFactory (ROS required)
│   ├── test_message_factory_standalone.cpp← MessageFactory (no ROS)
│   ├── test_metadata_injector.cpp         ← MetadataInjector (42 tests)
│   ├── test_recorder_context_interface.cpp← IRecorderContext (19 tests)
│   ├── test_recording_service_impl.cpp    ← RecordingServiceImpl (51 tests)
│   ├── test_recording_session.cpp         ← RecordingSession (24 tests)
│   ├── test_ros_introspection.cpp         ← ROS introspection (17 tests)
│   ├── test_ros_sink.cpp                  ← AxonRosSink logging (16 tests)
│   ├── test_spsc_queue.cpp                ← Lock-free queue (23 tests)
│   ├── test_state_machine.cpp             ← StateManager (36 tests)
│   ├── test_state_transaction_guard.cpp   ← StateTransactionGuard (8 tests)
│   ├── test_task_config.cpp               ← TaskConfig (14 tests)
│   ├── test_topic_manager.cpp             ← TopicManager (24 tests)
│   └── test_worker_thread_pool.cpp        ← WorkerThreadPool (31 tests)
│
├── integration/                       # GTest + real ROS (6 test files)
│   ├── test_metadata_injection.cpp        ← Metadata injection flow (6 tests)
│   ├── test_recorder_node_lifecycle.cpp   ← RecorderNode lifecycle (17 tests)
│   ├── test_recorder_node_recording.cpp   ← RecorderNode recording (19 tests)
│   ├── test_recording_workflow.cpp        ← Full workflow (20 tests)
│   ├── test_ros_interface.cpp             ← RosInterface (23 tests)
│   └── test_service_adapter.cpp           ← ServiceAdapter (15 tests)
│
├── e2e/                               # E2E shell scripts (Humble only)
│   ├── run_e2e_tests.sh                   ← E2E test runner
│   └── test_ros_services.sh               ← ROS service E2E tests
│
├── stress/                            # Stress and load tests
│   ├── test_recording_session_stress.cpp  ← High-throughput write tests
│   └── test_recording_service_impl_stress.cpp ← Concurrent service tests
│
├── perf/                              # Performance benchmarks
│   ├── CMakeLists.txt                     ← Perf test build config
│   ├── perf_test_node.cpp                 ← Performance test node
│   ├── synthetic_publisher.cpp            ← Test data publisher
│   └── run_perf_test.sh                   ← Perf test runner
│
└── regression/                        # Bug reproduction tests (1 file)
    └── test_error_recovery.cpp            ← Error recovery (23 tests)
```

**Structure Notes:**
- E2E shell scripts are in dedicated `e2e/` folder (run on Humble only)
- GTest integration tests remain in `integration/` folder (run on all distros)
- Mocks are co-located with unit tests (not in a separate `mocks/` subfolder)
- Auto-discovery is already implemented via `axon_discover_unit_tests` macro

#### Future Structure Improvements (Optional)

Consider these improvements if the test suite grows significantly:

```
test/
├── unit/
│   ├── mocks/                         # Consolidate mock headers
│   │   ├── mock_recorder_context.hpp
│   │   └── mock_ros_interface_logging.hpp
│   └── ... (test files)
│
└── stress/                            # Add stress tests when needed
    ├── test_recording_session_stress.cpp
    └── test_high_throughput.cpp
```

> **Note:** The E2E scripts were renamed from `run_integration_tests.sh` to `run_e2e_tests.sh` and moved to `test/e2e/` to better reflect their purpose (end-to-end tests via ROS service calls, not GTest integration tests).

#### Test Category Definitions

| Folder | Test Type | Invocation | ROS Required | CI Distributions | Typical Runtime |
|--------|-----------|------------|--------------|------------------|-----------------|
| `unit/` | Unit tests | `ctest -L unit` | No | All (Noetic, Humble, Jazzy, Rolling) | <1ms per test |
| `integration/` | Integration tests | `ctest -L integration` | Yes | All (Noetic, Humble, Jazzy, Rolling) | 100ms-1s per test |
| `stress/` | Stress tests | `ctest -L stress` | Partial | Humble only | 5-60s per test |
| `e2e/` | E2E tests | `./run_e2e_tests.sh` | Yes | **Humble only** | 5-30s |
| `perf/` | Performance tests | `./run_perf_test.sh` | Yes | Humble only | 1-5 min |
| `regression/` | Regression tests | `ctest` | Varies | All | Varies |

> **Note:** E2E tests run only on Humble LTS because unit and integration tests already verify ROS API compatibility across all distributions. This saves ~10 minutes of CI time per PR.

#### Key Distinctions

| Aspect | Integration Test | E2E Test (Shell Scripts) |
|--------|------------------|--------------------------|
| **Entry Point** | GTest `main()` | Shell script |
| **Node Control** | Test creates/controls node | Separate running node |
| **Interface** | C++ method calls | ROS service calls |
| **Scope** | Component interactions | Full system behavior |
| **Example** | `node->start_recording()` | `ros2 service call .../start` |

#### CMakeLists.txt: Auto-Discovery Already Implemented ✓

The test infrastructure already uses auto-discovery via the `axon_discover_unit_tests` macro:

```cmake
# From test/CMakeLists.txt (lines 122-165)

macro(axon_discover_unit_tests)
    cmake_parse_arguments(DISCOVER "" "DIR;LABEL" "LIBS;EXCLUDE" ${ARGN})
    
    # Find all test_*.cpp files in the directory
    file(GLOB _DISCOVER_TEST_SOURCES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/${DISCOVER_DIR}/test_*.cpp"
    )
    
    # Exclude specific files if requested
    if(DISCOVER_EXCLUDE)
        foreach(_EXCLUDE_PATTERN ${DISCOVER_EXCLUDE})
            list(FILTER _DISCOVER_TEST_SOURCES EXCLUDE REGEX "${_EXCLUDE_PATTERN}")
        endforeach()
    endif()
    
    # Register each test with gtest_discover_tests
    foreach(_TEST_SOURCE ${_DISCOVER_TEST_SOURCES})
        get_filename_component(_TEST_NAME ${_TEST_SOURCE} NAME_WE)
        add_executable(${_TEST_NAME} ${_TEST_SOURCE})
        # ... linking and configuration ...
        gtest_discover_tests(${_TEST_NAME}
            PROPERTIES LABELS "${DISCOVER_LABEL}"
            PROPERTIES TIMEOUT ${AXON_TEST_TIMEOUT}
        )
    endforeach()
endmacro()
```

**Current Usage:**

```cmake
# Unit tests - auto-discover all test_*.cpp in unit/
axon_discover_unit_tests(
    DIR unit
    LIBS axon_recorder_test_support axon_logging
    LABEL unit
    EXCLUDE "test_ros_sink"           # Needs extra source file
            "test_ros_introspection"  # Needs ROS compile definitions
            "test_recording_session"  # Needs extra sources
            "test_metadata_injector"  # Needs extra sources
            "test_message_factory"    # Requires ROS
)
```

**Adding a New Unit Test:**
```bash
# Just create the file - CMake auto-discovers it on next configure
touch test/unit/test_my_feature.cpp
# No CMakeLists.txt edit needed!
```

**Running Tests by Category:**
```bash
# Run only unit tests (fast)
ctest -L unit --output-on-failure

# Run all tests
ctest --output-on-failure

# Run specific test
ctest -R test_state_machine --output-on-failure
```

---

## 4. Detailed Test Plans

### 4.1 Existing Test Coverage Summary

#### RecorderNode Tests (Existing)

**`test/integration/test_recorder_node_lifecycle.cpp`** (17 tests) - ✅ EXISTS
- Factory method (`CreateRecorderNode`)
- State queries before recording
- Shutdown safety (before init, double shutdown)
- Recording state methods (is_recording, is_paused, etc.)

**`test/integration/test_recorder_node_recording.cpp`** (19 tests) - ✅ EXISTS
- Recording workflow (start/stop/pause/resume)
- Duration tracking
- Statistics reporting

#### RecordingSession Tests (Existing)

**`test/unit/test_recording_session.cpp`** (24 tests) - ✅ EXISTS
- Open/close lifecycle
- Schema registration
- Channel registration
- Message writing
- Statistics

**`test/integration/test_metadata_injection.cpp`** (6 tests) - ✅ EXISTS
- Metadata injection flow
- Task config integration

#### RecordingServiceImpl Tests (Existing)

**`test/unit/test_recording_service_impl.cpp`** (51 tests) - ✅ EXISTS
- All service handlers (CachedRecordingConfig, IsRecordingReady, RecordingControl, RecordingStatus)
- State machine integration
- Error handling
- Concurrency tests

### 4.2 Additional Tests (Gap Coverage) - ✅ IMPLEMENTED

#### 4.2.1 RecorderNode: Edge Uploader Tests - ✅ IMPLEMENTED

**File:** `test/integration/test_recorder_node_uploader.cpp`

> **Note:** These tests require `AXON_HAS_UPLOADER` to be defined. They conditionally compile.

```cpp
#ifdef AXON_HAS_UPLOADER
TEST_F(RecorderNodeUploaderTest, UploaderInitializedWhenEnabled)      // ✅
TEST_F(RecorderNodeUploaderTest, UploaderNotInitializedWhenDisabled)  // ✅
TEST_F(RecorderNodeUploaderTest, FileQueuedAfterStop)                 // ✅
TEST_F(RecorderNodeUploaderTest, CorruptFileNotQueued)                // ✅
TEST_F(RecorderNodeUploaderTest, UploaderShutdownClean)               // ✅
TEST_F(RecorderNodeUploaderTest, HealthStatusReportsCorrectly)        // ✅
TEST_F(RecorderNodeUploaderTest, MultipleFilesEnqueuedInOrder)        // ✅
TEST_F(RecorderNodeUploaderTest, CallbackInvokedOnCompletion)         // ✅
#endif
```

#### 4.2.2 RecordingSession: Extended Metadata Tests - ✅ IMPLEMENTED

**Added to `test/unit/test_recording_session.cpp`:**

```cpp
// Metadata Injection
TEST_F(RecordingSessionTest, SetTaskConfigEnablesMetadata)     // ✅
TEST_F(RecordingSessionTest, SidecarPathAvailableAfterClose)   // ✅
TEST_F(RecordingSessionTest, ChecksumAvailableAfterClose)      // ✅

// Topic Statistics
TEST_F(RecordingSessionTest, UpdateTopicStatsAccumulates)      // ✅
TEST_F(RecordingSessionTest, MessageTypeStoredOnFirstUpdate)   // ✅
```

#### 4.2.3 Stress Tests - ✅ IMPLEMENTED

**Directory:** `test/stress/`

**File: `test/stress/test_recording_session_stress.cpp`**

```cpp
TEST_F(RecordingSessionStressTest, ConcurrentWriteFrom8Threads)      // ✅
TEST_F(RecordingSessionStressTest, Write100KMessagesPerSecond)       // ✅
TEST_F(RecordingSessionStressTest, LargeMessagesMemoryStable)        // ✅
TEST_F(RecordingSessionStressTest, MixedMessageSizesConcurrent)      // ✅
TEST_F(RecordingSessionStressTest, SustainedLoadFor5Seconds)         // ✅
```

**File: `test/stress/test_recording_service_impl_stress.cpp`**

```cpp
TEST_F(RecordingServiceImplStressTest, RapidStateChanges100PerSecond)         // ✅
TEST_F(RecordingServiceImplStressTest, ConcurrentStatusQueriesWhileRecording) // ✅
TEST_F(RecordingServiceImplStressTest, RapidConfigCaching)                    // ✅
TEST_F(RecordingServiceImplStressTest, ConcurrentIsRecordingReadyQueries)     // ✅
TEST_F(RecordingServiceImplStressTest, MixedCommandsFromMultipleThreads)      // ✅
```

**Running Stress Tests:**
```bash
# Run all stress tests
ctest -L stress --output-on-failure

# Run specific stress test
ctest -R test_recording_session_stress --output-on-failure
```

---

## 5. Test Infrastructure

### 5.1 Existing Mock Infrastructure

**`test/unit/mock_recorder_context.hpp`** - ✅ EXISTS

```cpp
/**
 * MockRecorderContext for testing RecordingServiceImpl without RecorderNode.
 * Implements IRecorderContext interface for dependency injection.
 */
class MockRecorderContext : public IRecorderContext {
public:
  // State management
  StateManager& get_state_manager() override;
  TaskConfigCache& get_task_config_cache() override;
  
  // Recording operations
  bool start_recording() override;
  void stop_recording() override;
  void pause_recording() override;
  void resume_recording() override;
  void cancel_recording() override;
  
  // Verification helpers
  bool was_start_recording_called() const;
  bool was_stop_recording_called() const;
  // ... etc
};
```

**`test/unit/mock_ros_interface_logging.hpp`** - ✅ EXISTS

Mock ROS logging interface for testing logging components.

### 5.2 Test Support Library

The `axon_recorder_test_support` static library compiles common sources once for all tests:

```cmake
# From test/CMakeLists.txt
add_library(axon_recorder_test_support STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/state_machine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/config_parser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/http_callback_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/topic_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/worker_thread_pool.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/recording_service_impl.cpp
)
```

### 5.3 Test Fixture Helpers - ✅ IMPLEMENTED

**File:** `test/unit/test_helpers.hpp`

```cpp
namespace axon::recorder::test {

// Standard test configurations
TaskConfig make_test_task_config();
TaskConfig make_minimal_task_config(const std::string& task_id = "minimal_task");
RecorderConfig make_test_recorder_config(const std::string& output_dir);
RecorderConfig make_minimal_recorder_config(const std::string& output_dir, const std::string& topic_name = "/test_topic");

// Temp directory management (RAII)
class TempDirectory {
public:
  TempDirectory();
  ~TempDirectory();  // Cleanup on destruction
  std::filesystem::path path() const;
  std::filesystem::path subdir(const std::string& name) const;
  std::filesystem::path file(const std::string& name) const;
};

// Test utilities
std::vector<uint8_t> make_test_message(size_t size);
uint64_t now_ns();

}  // namespace axon::recorder::test
```

> **Design Decision:** Real MCAP code is used directly in tests rather than mocking `McapWriterWrapper`. This ensures tests validate actual MCAP file generation and avoids maintenance overhead of keeping mocks in sync with implementation changes.

---

## 6. Implementation Priority

### Current Status

With **~610+ tests** across 25+ files, the test suite has comprehensive coverage. All proposed gaps have been implemented.

### Phase 1: Metadata Injection Coverage (P1) - ✅ COMPLETED

| Task | Description | Status |
|------|-------------|--------|
| Add metadata tests to `test_recording_session.cpp` | Cover `set_task_config()`, sidecar generation | ✅ Done |
| 5 new metadata tests added | SetTaskConfigEnablesMetadata, SidecarPathAvailableAfterClose, etc. | ✅ Done |

### Phase 2: Edge Uploader Testing (P1) - ✅ COMPLETED

| Task | Description | Status |
|------|-------------|--------|
| Create `test_recorder_node_uploader.cpp` | Test `#ifdef AXON_HAS_UPLOADER` paths | ✅ Done |
| 8 uploader integration tests | Conditional compilation, queuing, shutdown | ✅ Done |

### Phase 3: Stress Testing (P2) - ✅ COMPLETED

| Task | Description | Status |
|------|-------------|--------|
| Create `test/stress/` directory | Organize stress tests | ✅ Done |
| `test_recording_session_stress.cpp` | High-throughput write tests (5 tests) | ✅ Done |
| `test_recording_service_impl_stress.cpp` | Concurrent service call tests (5 tests) | ✅ Done |
| CMakeLists.txt stress test registration | Auto-discovery with stress label | ✅ Done |

### Phase 4: Test Helpers Infrastructure - ✅ COMPLETED

| Task | Description | Status |
|------|-------------|--------|
| Create `test/unit/test_helpers.hpp` | TempDirectory, config helpers | ✅ Done |
| Common test utilities | make_test_message(), now_ns() | ✅ Done |

### All Completed ✓

- ✅ Auto-discovery infrastructure (`axon_discover_unit_tests` macro)
- ✅ MockRecorderContext for service testing
- ✅ Test support library (`axon_recorder_test_support`)
- ✅ Comprehensive unit tests for all core components
- ✅ Integration tests for RecorderNode lifecycle
- ✅ Performance test infrastructure (`test/perf/`)
- ✅ **Metadata injection tests (5 tests)**
- ✅ **Edge uploader integration tests (8 tests, conditional)**
- ✅ **Stress tests (10 tests)**
- ✅ **Test fixture helpers (`test_helpers.hpp`)**

---

## 7. Test Execution Matrix

### ROS Distribution × Test Category

| Test Category | Noetic | Humble | Jazzy | Rolling |
|---------------|--------|--------|-------|---------|
| Unit Tests (~480) | ✓ | ✓ | ✓ | ✓ |
| Integration Tests (~110) | ✓ | ✓ | ✓ | ✓ |
| Stress Tests (10) | - | ✓ | - | - |
| Regression Tests (23) | ✓ | ✓ | ✓ | ✓ |
| E2E Scripts | - | ✓ | - | - |
| Performance Tests | - | ✓ | ○ | - |

Legend: ✓ = Required, ○ = Optional, - = Not run

**Notes:**
- Unit tests run without ROS runtime
- Integration tests require ROS environment
- Stress tests run on Humble only (`ctest -L stress`)
- E2E shell scripts only run on Humble LTS
- Performance tests are manual (run via `./test/perf/run_perf_test.sh`)

### CI Pipeline

```
┌─────────────────────────────────────────────────┐
│               Unit Tests (Stage 1)              │
│  ┌───────────────────┐  ┌───────────────────┐   │
│  │ ROS Unit Tests    │  │ C++ Unit Tests    │   │  (parallel)
│  │ +cov (Humble/     │  │ +cov              │   │
│  │      Noetic)      │  │                   │   │
│  └───────────────────┘  └───────────────────┘   │
└─────────────────────┬───────────────────────────┘
                      │
             ┌────────┴────────┐
             ▼                 ▼
       ┌───────────┐     ┌───────────┐
       │ Integr.   │     │   E2E     │  (parallel, both collect coverage)
       │  Tests    │     │   Tests   │
       │  +cov     │     │   +cov    │
       └───────────┘     └───────────┘
```

| Workflow | File | Runs On | Tests | Coverage |
|----------|------|---------|-------|----------|
| ROS Unit Tests | `unit-tests-ros.yml` | All distros | `ctest -LE integration` | Humble/Noetic → `ros1`/`ros2` |
| C++ Unit Tests | `unit-tests-cpp.yml` | Ubuntu 22.04 | C++ library tests | → `cpp` |
| Integration Tests | `integration-tests.yml` | All distros | `ctest -L integration` | Humble/Noetic → `ros1`/`ros2` |
| E2E Tests | `e2e-tests.yml` | Humble only | `run_e2e_tests.sh` | → `ros2` |

### CI Test Commands

```bash
# Run all tests
colcon test --packages-select axon_recorder

# Run unit tests only (fast)
ctest -L unit --output-on-failure

# Run integration tests only
ctest -L integration --output-on-failure

# Run specific test file
ctest -R test_state_machine --output-on-failure

# Run with verbose output
ctest --output-on-failure --verbose
```

### Docker Test Environments

The project provides Docker configurations for all supported ROS distributions:

```
ros/docker/
├── Dockerfile.ros1          # ROS1 Noetic
├── Dockerfile.ros2.humble   # ROS2 Humble LTS
├── Dockerfile.ros2.jazzy    # ROS2 Jazzy
├── Dockerfile.ros2.rolling  # ROS2 Rolling
└── docker-compose.test.yml  # Test orchestration
```

---

## Appendices

### A. Test Naming Conventions

```
TEST_F(<Fixture>Test, <Method>_<Scenario>_<Expected>)
TEST_F(<Fixture>Test, <Scenario>_<Expected>)

Examples:
TEST_F(RecorderNodeTest, Initialize_WithInvalidConfig_ReturnsFalse)
TEST_F(RecordingSessionTest, Write_WhenClosed_ReturnsFalse)
TEST_F(RecordingServiceImplTest, StartCommand_WhenNotReady_FailsWithError)
```

### B. Test Labels

| Label | Description | Run Command |
|-------|-------------|-------------|
| `unit` | Fast, no ROS required | `ctest -L unit` |
| `integration` | Requires ROS | `ctest -L integration` |

### C. Coverage Report Commands

```bash
# Build with coverage enabled
colcon build --packages-select axon_recorder --cmake-args -DENABLE_COVERAGE=ON

# Run tests
colcon test --packages-select axon_recorder

# Generate coverage report (from project root)
cd coverage/
lcov --capture --directory ../build --output-file coverage_raw.info
lcov --remove coverage_raw.info '/usr/*' '*/test/*' --output-file coverage.info
genhtml coverage.info --output-directory html
```

### D. Mock Interface Quick Reference

| Mock Class | File | Purpose |
|------------|------|---------|
| `MockRecorderContext` | `test/unit/mock_recorder_context.hpp` | Test RecordingServiceImpl without RecorderNode |
| `MockRosInterfaceLogging` | `test/unit/mock_ros_interface_logging.hpp` | Test logging components |

### E. Test File Inventory

**Total: ~620+ tests across 26 files**

| Category | Files | Tests |
|----------|-------|-------|
| Unit | 17 | ~480 |
| Integration | 7 | ~110 |
| Stress | 2 | 10 |
| Regression | 1 | 23 |

#### New Files Added (v1.4)

| File | Category | Tests |
|------|----------|-------|
| `test/unit/test_helpers.hpp` | Infrastructure | N/A |
| `test/stress/test_recording_session_stress.cpp` | Stress | 5 |
| `test/stress/test_recording_service_impl_stress.cpp` | Stress | 5 |
| `test/integration/test_recorder_node_uploader.cpp` | Integration | 8 |

#### Updated Files (v1.4)

| File | Tests Added |
|------|-------------|
| `test/unit/test_recording_session.cpp` | +5 metadata tests |

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-25 | Axon Robotics | Initial version |
| 1.1 | 2025-12-25 | Axon Robotics | Updated to reflect actual test structure and counts |
| 1.2 | 2025-12-25 | Axon Robotics | Renamed integration-tests.yml → e2e-tests.yml; E2E runs on Humble only; moved E2E scripts to test/e2e/ |
| 1.3 | 2025-12-25 | Axon Robotics | Added integration-tests.yml for GTest integration tests on all distros |
| 1.4 | 2025-12-25 | Axon Robotics | Implemented all proposed tests: metadata tests (5), stress tests (10), uploader tests (8), test_helpers.hpp; removed MockMcapWriterWrapper proposal (use real code); updated test counts |
| 1.5 | 2025-12-25 | Axon Robotics | CI refactor: unit-tests.yml → unit-tests-ros.yml; added unit-tests-cpp.yml; coverage now collected in all test workflows; removed standalone coverage-ros.yml |
| 1.6 | 2025-12-25 | Axon Robotics | Added Root Cause Analysis section for low coverage; fixed ROS2 bug (missing `register_common_message_types()` call); documented coverage collection strategy |

---

## Sign-off

| Role | Name | Date | Approval |
|------|------|------|----------|
| Author | | 2025-12-25 | |
| Reviewer (Staff Eng) | | | |
| Reviewer (Test Lead) | | | |


# Test Design Document: Axon Recorder Core Components

**Date:** 2026-01-12
**Document Version:** 3.0

## Executive Summary

This document provides comprehensive test design for Axon Recorder. Axon is a high-performance ROS recorder featuring a **plugin-based architecture** and **HTTP RPC API** for remote control.

### Architecture Changes (v3.0)

Compared to the previous ROS service architecture, the current implementation has major changes:

1. **Plugin-based Architecture** - Middleware-agnostic core + ROS1/ROS2 plugins
2. **HTTP RPC API** - Replaces ROS service interfaces
3. **Unified State Machine** - IDLE → READY → RECORDING ↔ PAUSED
4. **Dynamic Plugin Loading** - Runtime middleware plugin loading

### Core Components

| Component | File | Responsibility | Lines |
|-----------|------|-----------------|-------|
| **AxonRecorder** | [recorder.cpp/hpp](../apps/axon_recorder/recorder.cpp) | Main coordinator | ~350 |
| **HttpServer** | [http_server.cpp/hpp](../apps/axon_recorder/http_server.cpp) | HTTP RPC API | ~570 |
| **PluginLoader** | [plugin_loader.cpp/hpp](../apps/axon_recorder/plugin_loader.cpp) | Plugin loading | ~140 |
| **StateManager** | [state_machine.cpp/hpp](../apps/axon_recorder/state_machine.cpp) | State machine | ~200 |
| **RecordingSession** | [recording_session.cpp/hpp](../apps/axon_recorder/recording_session.cpp) | MCAP session | ~200 |
| **WorkerThreadPool** | [worker_thread_pool.cpp/hpp](../apps/axon_recorder/worker_thread_pool.cpp) | Worker thread pool | ~270 |
| **ConfigParser** | [config_parser.cpp/hpp](../apps/axon_recorder/config_parser.cpp) | Config parsing | ~370 |
| **MetadataInjector** | [metadata_injector.cpp/hpp](../apps/axon_recorder/metadata_injector.cpp) | Metadata injection | ~400 |
| **HttpCallbackClient** | [http_callback_client.cpp/hpp](../apps/axon_recorder/http_callback_client.cpp) | HTTP callbacks | ~360 |

### Testing Objectives

The test suite needs to be designed from scratch because:

1. **New HTTP RPC API** - Complete HTTP endpoint testing needed
2. **Plugin System** - Plugin loading and ABI compatibility testing needed
3. **State Machine** - Complete state transition testing needed
4. **Concurrency Model** - Thread safety and race condition testing needed

**Goal:** ~240 tests, coverage >85%

---

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Component Analysis](#2-component-analysis)
3. [Test Strategy](#3-test-strategy)
4. [HTTP RPC API Tests](#4-http-rpc-api-tests)
5. [Plugin System Tests](#5-plugin-system-tests)
6. [State Machine Tests](#6-state-machine-tests)
7. [Concurrency Tests](#7-concurrency-tests)
8. [Integration Tests](#8-integration-tests)
9. [E2E Tests](#9-e2e-tests)
10. [Test Infrastructure](#10-test-infrastructure)
11. [Implementation Priority](#11-implementation-priority)

---

## 1. Architecture Overview

### 1.1 System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         HTTP Clients                                │
│                    (curl, Postman, Browsers)                        │
└─────────────────────────────────────┬───────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      HttpServer (port 8080)                        │
│                   Boost.Beast HTTP + JSON                           │
└─────────────────────────────────────┬───────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       AxonRecorder                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐   │
│  │ StateMachine │  │ PluginLoader │  │    WorkerThreadPool     │   │
│  └──────┬───────┘  └──────┬───────┘  └────────────┬───────────┘   │
│         │                 │                        │               │
│         ▼                 ▼                        ▼               │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐   │
│  │ Recording    │  │ ROS2 Plugin  │  │   SPSC Queues          │   │
│  │ Session      │  │ (.so)        │  │   (per-topic)          │   │
│  └──────────────┘  └──────────────┘  └────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Core Libraries                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐   │
│  │ axon_mcap    │  │ axon_logging │  │   axon_uploader        │   │
│  │ (no ROS deps)│  │ (no ROS deps)│  │   (no ROS deps)        │   │
│  └──────────────┘  └──────────────┘  └────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 HTTP RPC Endpoints

| Method | Endpoint | State Transition | Description |
|--------|----------|------------------|-------------|
| POST | `/rpc/config` | IDLE → READY | Set task configuration |
| POST | `/rpc/begin` | READY → RECORDING | Start recording |
| POST | `/rpc/finish` | RECORDING/PAUSED → IDLE | Finish recording |
| POST | `/rpc/pause` | RECORDING → PAUSED | Pause recording |
| POST | `/rpc/resume` | PAUSED → RECORDING | Resume recording |
| POST | `/rpc/cancel` | RECORDING/PAUSED → IDLE | Cancel recording |
| POST | `/rpc/clear` | READY → IDLE | Clear configuration |
| POST | `/rpc/quit` | Any → Exit | Exit program |
| GET | `/rpc/state` | - | Get current state |
| GET | `/rpc/stats` | - | Get statistics |
| GET | `/` or `/health` | - | Health check |

See [rpc-api-design.md](./rpc-api-design.md) for detailed API specification.

### 1.3 Plugin ABI Interface

Plugins expose functionality through the following C structures:

```cpp
// Plugin descriptor
struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

// Plugin function table
struct AxonPluginVtable {
  AxonInitFn init;           // Initialize plugin
  AxonStartFn start;         // Start plugin
  AxonStopFn stop;           // Stop plugin
  AxonSubscribeFn subscribe; // Subscribe to topics
  AxonPublishFn publish;     // Publish messages
  void* reserved[9];
};
```

---

## 2. Component Analysis

### 2.1 AxonRecorder (recorder.cpp)

**Responsibilities:**
- Plugin loading and lifecycle management
- HTTP RPC server management
- State machine coordination
- Worker thread pool management
- MCAP session management
- Metadata injection

**Key Methods:**

| Method | Complexity | Test Priority |
|--------|------------|---------------|
| `initialize()` | High | P0 |
| `start()` | High | P0 |
| `stop()` | High | P0 |
| `set_task_config()` | Low | P1 |
| `on_message()` | Medium | P0 |
| `start_http_server()` | Medium | P0 |
| `message_handler()` | Medium | P1 |
| `register_topics()` | Medium | P1 |

**Test Coverage Focus:**
- Initialization failure scenarios (plugin not found, config error)
- Correctness of state transitions
- Concurrent message processing
- Resource cleanup (destructor, stop)

### 2.2 HttpServer (http_server.cpp)

**Responsibilities:**
- HTTP request handling
- JSON request/response parsing
- RPC endpoint routing
- Error handling

**Key Methods:**

| Method | Complexity | Test Priority |
|--------|------------|---------------|
| `handle_request()` | High | P0 |
| `handle_rpc_begin()` | Medium | P0 |
| `handle_rpc_finish()` | Medium | P0 |
| `handle_rpc_set_config()` | Medium | P0 |
| `handle_rpc_get_state()` | Low | P0 |
| `handle_rpc_get_stats()` | Low | P1 |
| `handle_rpc_pause()` | Low | P2 |
| `handle_rpc_resume()` | Low | P2 |
| `handle_rpc_cancel()` | Low | P2 |
| `handle_rpc_clear()` | Low | P2 |
| `handle_rpc_quit()` | High | P0 |

**Test Coverage Focus:**
- All RPC endpoint happy paths
- State transition errors (called from wrong state)
- Invalid JSON requests
- Concurrent HTTP requests
- Server start/stop

### 2.3 PluginLoader (plugin_loader.cpp)

**Responsibilities:**
- Dynamic library loading (dlopen)
- Symbol resolution (dlsym)
- Plugin lifecycle management
- ABI version verification

**Key Methods:**

| Method | Complexity | Test Priority |
|--------|------------|---------------|
| `load()` | High | P0 |
| `unload()` | Medium | P0 |
| `get_descriptor()` | Low | P0 |
| `get_plugin()` | Low | P1 |

**Test Coverage Focus:**
- Successful load of valid plugin
- Load non-existent library
- Load invalid plugin (missing symbols)
- ABI version mismatch
- Duplicate loading
- Unload and resource cleanup

### 2.4 StateManager (state_machine.cpp)

**Responsibilities:**
- State transition management
- Transition validation
- Transition callbacks

**State Transition Diagram:**
```
IDLE → READY → RECORDING ↔ PAUSED
  ↑                         ↓
  └───────── (finish/cancel)
```

**Test Coverage Focus:**
- All valid state transitions
- Invalid state transitions
- Transition callback triggering
- Concurrent transition attempts

### 2.5 RecordingSession (recording_session.cpp)

**Responsibilities:**
- MCAP file open/close
- Schema and channel registration
- Message writing
- Statistics collection

**Test Coverage Focus:**
- File lifecycle
- Schema registration failure
- Duplicate registration handling
- Concurrent writes
- Metadata injection

### 2.6 WorkerThreadPool (worker_thread_pool.cpp)

**Responsibilities:**
- Worker thread creation/destruction
- SPSC queue management
- Message dispatch

**Test Coverage Focus:**
- Thread creation/destruction
- Behavior when queue is full
- Message ordering
- Graceful shutdown

---

## 3. Test Strategy

### 3.1 Test Pyramid

```
                    ╭─────────────────╮
                    │   E2E Tests     │  ← HTTP API complete workflows
                    │   (~20 tests)   │
                ╭───┴─────────────────┴───╮
                │  Integration Tests      │  ← Component integration
                │  (~60 tests)            │
            ╭───┴─────────────────────────┴───╮
            │        Unit Tests               │  ← Single components
            │        (~150 tests)            │
        ╭───┴─────────────────────────────────┴───╮
        │     Core Libraries (axon_*)             │
        │     (independent tests, existing)       │
        ╰─────────────────────────────────────────╯
```

### 3.2 Test Categories

| Category | Scope | Framework | ROS Required | Typical Runtime |
|----------|-------|-----------|--------------|-----------------|
| **Unit** | Single component, isolated deps | GTest | No | <10ms |
| **Integration** | Multi-component collaboration | GTest | No* | 50-200ms |
| **HTTP API** | HTTP endpoint testing | GTest + httplib | No | 20-100ms |
| **E2E** | Complete recording workflow | Shell/Python | Yes* | 1-10s |
| **Stress** | High concurrency, long duration | GTest | No | 5-60s |
| **Performance** | Throughput, latency | Custom | Yes | 1-5min |

*Note: Integration tests can avoid ROS dependencies using mock plugins; E2E tests require real ROS environment.

### 3.3 Test Coverage Targets

| Component | Line Coverage | Branch Coverage |
|-----------|---------------|-----------------|
| AxonRecorder | >90% | >85% |
| HttpServer | >95% | >90% |
| PluginLoader | >90% | >85% |
| StateManager | >95% | >90% |
| RecordingSession | >90% | >85% |
| WorkerThreadPool | >85% | >80% |
| ConfigParser | >90% | >85% |
| MetadataInjector | >85% | >80% |
| HttpCallbackClient | >80% | >75% |

---

## 4. HTTP RPC API Tests

### 4.1 Test File Structure

```
apps/axon_recorder/test/
├── CMakeLists.txt
├── unit/
│   ├── test_helpers.hpp
│   ├── test_http_server.cpp           # HTTP server unit tests (~40 tests)
│   ├── test_plugin_loader.cpp         # Plugin loader tests (~30 tests)
│   ├── test_state_machine.cpp         # State machine tests (~25 tests)
│   ├── test_recording_session.cpp     # MCAP session tests (~20 tests)
│   ├── test_worker_thread_pool.cpp    # Worker thread pool tests (~15 tests)
│   ├── test_config_parser.cpp         # Config parser tests (~15 tests)
│   └── test_task_config.cpp           # Task config tests (~10 tests)
├── integration/
│   ├── test_recorder_integration.cpp  # AxonRecorder integration tests (~25 tests)
│   ├── test_http_api_integration.cpp  # HTTP API integration tests (~30 tests)
│   └── test_plugin_integration.cpp    # Plugin integration tests (~15 tests)
└── e2e/
    ├── run_e2e_tests.sh               # E2E test script
    └── test_http_api_e2e.sh           # HTTP API E2E test
```

### 4.2 HTTP Server Unit Tests (test_http_server.cpp)

**Test Fixture:**
```cpp
class HttpServerTest : public ::testing::Test {
protected:
  void SetUp() override;
  void TearDown() override;

  axon::recorder::AxonRecorder* recorder_;
  axon::recorder::HttpServer* server_;
  std::string server_url_;
};
```

**Test Cases:**

#### 4.2.1 Server Lifecycle (5 tests)

```cpp
TEST_F(HttpServerTest, Start_BindsToPort)
TEST_F(HttpServerTest, Start_WhenAlreadyRunning_ReturnsFalse)
TEST_F(HttpServerTest, Stop_WhenRunning_StopsCleanly)
TEST_F(HttpServerTest, Stop_WhenNotRunning_NoOp)
TEST_F(HttpServerTest, GetUrl_ReturnsCorrectUrl)
```

#### 4.2.2 RPC: /rpc/config (8 tests)

```cpp
TEST_F(HttpServerTest, SetConfig_WithValidConfig_SetsConfigAndTransitionsToReady)
TEST_F(HttpServerTest, SetConfig_WithMissingTaskConfig_ReturnsError)
TEST_F(HttpServerTest, SetConfig_WithInvalidJson_ReturnsError)
TEST_F(HttpServerTest, SetConfig_WhenRecording_ReturnsError)
TEST_F(HttpServerTest, SetConfig_Twice_OverwritesPreviousConfig)
TEST_F(HttpServerTest, SetConfig_WithEmptyTopics_Accepts)
TEST_F(HttpServerTest, SetConfig_WithCallbackUrls_SetsUrls)
TEST_F(HttpServerTest, SetConfig_WithUserToken_SetsToken)
```

#### 4.2.3 RPC: /rpc/begin (8 tests)

```cpp
TEST_F(HttpServerTest, Begin_WhenReady_TransitionsToRecording)
TEST_F(HttpServerTest, Begin_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Begin_WhenRecording_ReturnsStateError)
TEST_F(HttpServerTest, Begin_WhenPaused_ReturnsStateError)
TEST_F(HttpServerTest, Begin_WithoutConfig_ReturnsError)
TEST_F(HttpServerTest, Begin_WithValidTaskId_StartsRecording)
TEST_F(HttpServerTest, Begin_WithInvalidTaskId_ReturnsError)
TEST_F(HttpServerTest, Begin_StartsPluginAndSubscribes)
```

#### 4.2.4 RPC: /rpc/finish (8 tests)

```cpp
TEST_F(HttpServerTest, Finish_WhenRecording_TransitionsToIdle)
TEST_F(HttpServerTest, Finish_WhenPaused_TransitionsToIdle)
TEST_F(HttpServerTest, Finish_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Finish_WhenReady_ReturnsStateError)
TEST_F(HttpServerTest, Finish_WithValidTaskId_ClosesSession)
TEST_F(HttpServerTest, Finish_WithInvalidTaskId_ReturnsError)
TEST_F(HttpServerTest, Finish_GeneratesSidecarAndChecksum)
TEST_F(HttpServerTest, Finish_CallsFinishCallback)
```

#### 4.2.5 RPC: /rpc/pause (5 tests)

```cpp
TEST_F(HttpServerTest, Pause_WhenRecording_TransitionsToPaused)
TEST_F(HttpServerTest, Pause_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Pause_WhenReady_ReturnsStateError)
TEST_F(HttpServerTest, Pause_WhenPaused_ReturnsStateError)
TEST_F(HttpServerTest, Pause_WhenNotImplemented_Returns501)
```

#### 4.2.6 RPC: /rpc/resume (5 tests)

```cpp
TEST_F(HttpServerTest, Resume_WhenPaused_TransitionsToRecording)
TEST_F(HttpServerTest, Resume_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Resume_WhenReady_ReturnsStateError)
TEST_F(HttpServerTest, Resume_WhenRecording_ReturnsStateError)
TEST_F(HttpServerTest, Resume_WhenNotImplemented_Returns501)
```

#### 4.2.7 RPC: /rpc/cancel (6 tests)

```cpp
TEST_F(HttpServerTest, Cancel_WhenRecording_TransitionsToIdle)
TEST_F(HttpServerTest, Cancel_WhenPaused_TransitionsToIdle)
TEST_F(HttpServerTest, Cancel_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Cancel_WhenReady_ReturnsStateError)
TEST_F(HttpServerTest, Cancel_WithValidTaskId_CancelsRecording)
TEST_F(HttpServerTest, Cancel_DoesNotGenerateSidecar)
```

#### 4.2.8 RPC: /rpc/clear (5 tests)

```cpp
TEST_F(HttpServerTest, Clear_WhenReady_TransitionsToIdle)
TEST_F(HttpServerTest, Clear_WhenIdle_ReturnsStateError)
TEST_F(HttpServerTest, Clear_WhenRecording_ReturnsStateError)
TEST_F(HttpServerTest, Clear_WhenPaused_ReturnsStateError)
TEST_F(HttpServerTest, Clear_RemovesTaskConfig)
```

#### 4.2.9 RPC: /rpc/quit (7 tests)

```cpp
TEST_F(HttpServerTest, Quit_WhenIdle_ExitsCleanly)
TEST_F(HttpServerTest, Quit_WhenReady_ExitsCleanly)
TEST_F(HttpServerTest, Quit_WhenRecording_StopsAndExits)
TEST_F(HttpServerTest, Quit_WhenPaused_StopsAndExits)
TEST_F(HttpServerTest, Quit_WithValidTaskId_StopsAndExits)
TEST_F(HttpServerTest, Quit_SendsFinishCallback)
TEST_F(HttpServerTest, Quit_StopsHttpServer)
```

#### 4.2.10 RPC: /rpc/state (6 tests)

```cpp
TEST_F(HttpServerTest, GetState_WhenIdle_ReturnsIdle)
TEST_F(HttpServerTest, GetState_WhenReady_ReturnsReadyWithConfig)
TEST_F(HttpServerTest, GetState_WhenRecording_ReturnsRecordingWithConfig)
TEST_F(HttpServerTest, GetState_WhenPaused_ReturnsPausedWithConfig)
TEST_F(HttpServerTest, GetState_DoesNotReturnCallbackUrls)
TEST_F(HttpServerTest, GetState_DoesNotReturnUserToken)
```

#### 4.2.11 RPC: /rpc/stats (5 tests)

```cpp
TEST_F(HttpServerTest, GetStats_WhenIdle_ReturnsZeroStats)
TEST_F(HttpServerTest, GetStats_WhenRecording_ReturnsCurrentStats)
TEST_F(HttpServerTest, GetStats_WhenPaused_ReturnsCurrentStats)
TEST_F(HttpServerTest, GetStats_UpdatesInRealTime)
TEST_F(HttpServerTest, GetStats_IncludesAllFields)
```

#### 4.2.12 Health Check (3 tests)

```cpp
TEST_F(HttpServerTest, HealthCheck_ReturnsOkStatus)
TEST_F(HttpServerTest, HealthCheck_IncludesVersion)
TEST_F(HttpServerTest, HealthCheck_IncludesCurrentState)
```

#### 4.2.13 Error Handling (8 tests)

```cpp
TEST_F(HttpServerTest, InvalidEndpoint_Returns404)
TEST_F(HttpServerTest, InvalidMethod_Returns405)
TEST_F(HttpServerTest, InvalidJson_Returns400)
TEST_F(HttpServerTest, EmptyRequest_Returns400)
TEST_F(HttpServerTest, MalformedJson_Returns400)
TEST_F(HttpServerTest, MissingContentType_Returns400)
TEST_F(HttpServerTest, ServerError_Returns500)
TEST_F(HttpServerTest, ConcurrentRequests_HandledCorrectly)
```

### 4.3 HTTP Integration Tests (test_http_api_integration.cpp)

**Complete workflow tests:**

```cpp
TEST(HttpApiIntegrationTest, CompleteRecordingWorkflow)
TEST(HttpApiIntegrationTest, MultipleRecordingTasks)
TEST(HttpApiIntegrationTest, PauseResumeWorkflow)
TEST(HttpApiIntegrationTest, CancelWorkflow)
TEST(HttpApiIntegrationTest, ClearConfigWorkflow)
TEST(HttpApiIntegrationTest, StateQueryAfterEachTransition)
TEST(HttpApiIntegrationTest, StatsDuringRecording)
TEST(HttpApiIntegrationTest, ConfigWithAllFields)
TEST(HttpApiIntegrationTest, ConfigWithMinimalFields)
TEST(HttpApiIntegrationTest, InvalidTransitions)
```

---

## 5. Plugin System Tests

### 5.1 Plugin Loader Tests (test_plugin_loader.cpp)

**Test Fixture:**
```cpp
class PluginLoaderTest : public ::testing::Test {
protected:
  void SetUp() override;
  void TearDown() override;

  std::string test_plugin_dir_;
  axon::recorder::PluginLoader loader_;
};
```

**Test Cases:**

#### 5.1.1 Load Valid Plugin (8 tests)

```cpp
TEST_F(PluginLoaderTest, Load_ValidPlugin_ReturnsSuccess)
TEST_F(PluginLoaderTest, Load_ValidPlugin_SetsDescriptor)
TEST_F(PluginLoaderTest, Load_ValidPlugin_CallsInit)
TEST_F(PluginLoaderTest, Load_ValidPlugin_SetsVtable)
TEST_F(PluginLoaderTest, Load_WithEmptyConfig_Succeeds)
TEST_F(PluginLoaderTest, Load_WithValidConfig_Succeeds)
TEST_F(PluginLoaderTest, Load_WithInvalidConfig_Fails)
TEST_F(PluginLoaderTest, Load_WithNullConfig_Fails)
```

#### 5.1.2 Load Invalid Plugin (10 tests)

```cpp
TEST_F(PluginLoaderTest, Load_NonExistentLibrary_ReturnsError)
TEST_F(PluginLoaderTest, Load_InvalidLibraryFormat_ReturnsError)
TEST_F(PluginLoaderTest, Load_MissingSymbol_ReturnsError)
TEST_F(PluginLoaderTest, Load_NullDescriptor_ReturnsError)
TEST_F(PluginLoaderTest, Load_AbiVersionMismatch_ReturnsError)
TEST_F(PluginLoaderTest, Load_MissingVtable_ReturnsError)
TEST_F(PluginLoaderTest, Load_MissingInitFunction_ReturnsError)
TEST_F(PluginLoaderTest, Load_CorruptedLibrary_ReturnsError)
TEST_F(PluginLoaderTest, Load_WrongArchitecture_ReturnsError)
TEST_F(PluginLoaderTest, Load_DependencyMissing_ReturnsError)
```

#### 5.1.3 Unload Plugin (6 tests)

```cpp
TEST_F(PluginLoaderTest, Unload_LoadedPlugin_Succeeds)
TEST_F(PluginLoaderTest, Unload_ByName_Succeeds)
TEST_F(PluginLoaderTest, Unload_NonExistentPlugin_ReturnsFalse)
TEST_F(PluginLoaderTest, Unload_All_Succeeds)
TEST_F(PluginLoaderTest, Unload_CallsStopIfRunning)
TEST_F(PluginLoaderTest, Unload_ClosesHandle)
```

#### 5.1.4 Query Plugin (6 tests)

```cpp
TEST_F(PluginLoaderTest, GetDescriptor_LoadedPlugin_ReturnsDescriptor)
TEST_F(PluginLoaderTest, GetDescriptor_NonExistentPlugin_ReturnsNull)
TEST_F(PluginLoaderTest, GetPlugin_LoadedPlugin_ReturnsPlugin)
TEST_F(PluginLoaderTest, GetPlugin_NonExistentPlugin_ReturnsNull)
TEST_F(PluginLoaderTest, IsLoaded_LoadedPlugin_ReturnsTrue)
TEST_F(PluginLoaderTest, LoadedPlugins_ReturnsListOfNames)
```

### 5.2 Mock Plugin

**Mock plugin implementation for testing:**

```cpp
// test/mock_plugin.cpp
extern "C" {

// Mock plugin descriptor
static axon::AxonPluginVtable g_vtable = {
  .init = [](const char* config) -> axon::AxonStatus {
    return AXON_SUCCESS;
  },
  .start = []() -> axon::AxonStatus {
    return AXON_SUCCESS;
  },
  .stop = []() -> axon::AxonStatus {
    return AXON_SUCCESS;
  },
  .subscribe = nullptr,
  .publish = nullptr,
};

static axon::AxonPluginDescriptor g_descriptor = {
  .abi_version_major = 1,
  .abi_version_minor = 0,
  .middleware_name = "mock",
  .middleware_version = "1.0.0",
  .plugin_version = "1.0.0",
  .vtable = &g_vtable,
};

const axon::AxonPluginDescriptor* axon_get_plugin_descriptor() {
  return &g_descriptor;
}

}  // extern "C"
```

---

## 6. State Machine Tests

### 6.1 State Machine Tests (test_state_machine.cpp)

**Test Cases:**

#### 6.1.1 Valid Transitions (10 tests)

```cpp
TEST(StateMachineTest, IdleToReady_WithConfig_Succeeds)
TEST(StateMachineTest, ReadyToRecording_Succeeds)
TEST(StateMachineTest, RecordingToPaused_Succeeds)
TEST(StateMachineTest, PausedToRecording_Succeeds)
TEST(StateMachineTest, RecordingToIdle_Succeeds)
TEST(StateMachineTest, PausedToIdle_Succeeds)
TEST(StateMachineTest, ReadyToIdle_WithClear_Succeeds)
TEST(StateMachineTest, RecordingToIdle_WithCancel_Succeeds)
TEST(StateMachineTest, PausedToIdle_WithCancel_Succeeds)
TEST(StateMachineTest, AnyToIdle_WithQuit_Succeeds)
```

#### 6.1.2 Invalid Transitions (10 tests)

```cpp
TEST(StateMachineTest, IdleToRecording_Fails)
TEST(StateMachineTest, IdleToPaused_Fails)
TEST(StateMachineTest, ReadyToIdle_Fails)
TEST(StateMachineTest, ReadyToPaused_Fails)
TEST(StateMachineTest, RecordingToReady_Fails)
TEST(StateMachineTest, RecordingToRecording_Fails)
TEST(StateMachineTest, PausedToReady_Fails)
TEST(StateMachineTest, PausedToPaused_Fails)
```

#### 6.1.3 Transition Callbacks (5 tests)

```cpp
TEST(StateMachineTest, Transition_CallsCallback)
TEST(StateMachineTest, Transition_WithMultipleCallbacks_CallsAll)
TEST(StateMachineTest, Transition_Failed_DoesNotCallCallback)
TEST(StateMachineTest, Callback_ReceivesFromAndToStates)
TEST(StateMachineTest, CanUnregisterCallback)
```

#### 6.1.4 State Queries (5 tests)

```cpp
TEST(StateMachineTest, GetState_ReturnsInitialState)
TEST(StateMachineTest, GetState_AfterTransition_ReturnsNewState)
TEST(StateMachineTest, GetStateString_ReturnsCorrectString)
TEST(StateMachineTest, IsRecording_WhenRecording_ReturnsTrue)
TEST(StateMachineTest, IsRecording_WhenNotRecording_ReturnsFalse)
```

---

## 7. Concurrency Tests

### 7.1 Concurrency Test Scenarios

| Scenario | Description | Test Method |
|----------|-------------|-------------|
| **Concurrent HTTP requests** | Multiple clients call API simultaneously | Multi-threaded HTTP client |
| **Concurrent message writes** | Multiple topics write simultaneously | Multi-threaded publishers |
| **State transition races** | Attempt state transitions simultaneously | Multi-threaded transition calls |
| **Queue overflow** | Fast publishers fill queue | High-frequency message generation |
| **Graceful shutdown** | Shutdown server while recording | Stop + resource check |

### 7.2 Concurrency Test Cases

**test_concurrency.cpp:**

```cpp
TEST(ConcurrencyTest, ConcurrentHttpRequests_HandledCorrectly)
TEST(ConcurrencyTest, ConcurrentMessageWrites_NoDataLoss)
TEST(ConcurrencyTest, ConcurrentStateTransitions_Serializes)
TEST(ConcurrencyTest, QueueFull_DropsMessages)
TEST(ConcurrencyTest, GracefulShutdown_WaitsForFlush)
TEST(ConcurrencyTest, RapidConfigChanges_HandlesCorrectly)
TEST(ConcurrencyTest, ConcurrentBeginFinish_Serializes)
TEST(ConcurrencyTest, MultipleClients_IndependentSessions)
```

### 7.3 Stress Tests

**test_stress.cpp:**

```cpp
TEST(StressTest, Write10000Messages_NoLoss)
TEST(StressTest, WriteFor60Seconds_MemoryStable)
TEST(StressTest, 100HttpConcurrentRequests_AllSucceed)
TEST(StressTest, 10ConfigChangesPerSecond_Handles)
TEST(StressTest, RapidStartStop_NoResourceLeak)
```

---

## 8. Integration Tests

### 8.1 AxonRecorder Integration Tests (test_recorder_integration.cpp)

**Complete workflow tests:**

```cpp
TEST(RecorderIntegrationTest, Initialize_Start_Stop)
TEST(RecorderIntegrationTest, ConfigAndBeginWorkflow)
TEST(RecorderIntegrationTest, RecordMessagesAndVerifyMcap)
TEST(RecorderIntegrationTest, PauseResumeWorkflow)
TEST(RecorderIntegrationTest, FinishAndGenerateSidecar)
TEST(RecorderIntegrationTest, MultipleRecordingTasks)
TEST(RecorderIntegrationTest, HttpServerIntegration)
TEST(RecorderIntegrationTest, PluginLoadAndSubscribe)
TEST(RecorderIntegrationTest, MetadataInjection)
TEST(RecorderIntegrationTest, StatisticsCollection)
TEST(RecorderIntegrationTest, ErrorHandling_InvalidPlugin)
TEST(RecorderIntegrationTest, ErrorHandling_DiskFull)
TEST(RecorderIntegrationTest, ErrorHandling_CorruptMcap)
```

### 8.2 Plugin Integration Tests (test_plugin_integration.cpp)

**Test integration with ROS2 plugin:**

```cpp
TEST(PluginIntegrationTest, LoadRos2Plugin_Succeeds)
TEST(PluginIntegrationTest, Ros2Plugin_Init_Succeeds)
TEST(PluginIntegrationTest, Ros2Plugin_Start_Succeeds)
TEST(PluginIntegrationTest, Ros2Plugin_Subscribe_ReceivesMessages)
TEST(PluginIntegrationTest, Ros2Plugin_Unsubscribe_StopsReceiving)
TEST(PluginIntegrationTest, Ros2Plugin_Stop_CleanShutdown)
TEST(PluginIntegrationTest, Ros2Plugin_ConcurrentSubscribes_Handles)
```

---

## 9. E2E Tests

### 9.1 E2E Test Script (test_http_api_e2e.sh)

```bash
#!/bin/bash
# E2E test script - Complete recording workflow

set -e

AXON_RECORDER_BIN="./build/axon_recorder"
PLUGIN_PATH="./middlewares/ros2/src/ros2_plugin/install/libaxon_ros2_plugin.so"
CONFIG_FILE="./test/e2e/test_config.yaml"

# Start recorder
$AXON_RECORDER_BIN --plugin $PLUGIN_PATH --config $CONFIG_FILE &
RECORDER_PID=$!

# Wait for startup
sleep 2

# 1. Health check
curl -f http://localhost:8080/health || exit 1

# 2. Set configuration
curl -X POST http://localhost:8080/rpc/config \
  -H "Content-Type: application/json" \
  -d '{
    "task_config": {
      "task_id": "e2e_test_001",
      "device_id": "test_robot",
      "topics": ["/test/topic1", "/test/topic2"]
    }
  }' || exit 1

# 3. Check state (should be READY)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "ready" ] || exit 1

# 4. Begin recording
curl -X POST http://localhost:8080/rpc/begin || exit 1

# 5. Check state (should be RECORDING)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "recording" ] || exit 1

# 6. Publish test messages
# (Use ROS2 to publish test data)

# 7. Wait for recording
sleep 5

# 8. Check statistics
curl -s http://localhost:8080/rpc/stats || exit 1

# 9. Finish recording
curl -X POST http://localhost:8080/rpc/finish \
  -H "Content-Type: application/json" \
  -d '{"task_id": "e2e_test_001"}' || exit 1

# 10. Check state (should be IDLE)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "idle" ] || exit 1

# 11. Verify MCAP file
[ -f "output.mcap" ] || exit 1

# 12. Verify sidecar file
[ -f "output.mcap.json" ] || exit 1

# Cleanup
kill $RECORDER_PID
wait $RECORDER_PID

echo "E2E tests passed!"
```

### 9.2 E2E Test Scenarios

| Scenario | Description | Validation Points |
|----------|-------------|-------------------|
| **Basic workflow** | config → begin → finish | MCAP file generated |
| **Pause/Resume** | begin → pause → resume → finish | Data integrity |
| **Cancel recording** | begin → cancel | No sidecar generated |
| **Clear config** | config → clear | Config cleared |
| **Multiple tasks** | Multiple begin/finish | Independent files each time |
| **Error recovery** | Invalid request handling | Correct error responses |

---

## 10. Test Infrastructure

### 10.1 Test Support Library

**test_helpers.hpp:**

```cpp
namespace axon::recorder::test {

// Test configuration
TaskConfig make_test_task_config();
RecorderConfig make_test_recorder_config();

// Temporary directory management
class TempDirectory {
public:
  TempDirectory();
  ~TempDirectory();
  std::filesystem::path path() const;
  std::filesystem::path file(const std::string& name) const;
};

// HTTP test client
class HttpClient {
public:
  HttpClient(const std::string& base_url);
  nlohmann::json post(const std::string& endpoint, const nlohmann::json& body);
  nlohmann::json get(const std::string& endpoint);
};

// MCAP verification
bool verify_mcap_file(const std::string& path);
bool verify_sidecar_file(const std::string& path);

}  // namespace axon::recorder::test
```

### 10.2 CMake Configuration

**test/CMakeLists.txt:**

```cmake
# Test executables
add_executable(test_http_server unit/test_http_server.cpp)
target_link_libraries(test_http_server
    axon_recorder
    gtest
    gtest_main
    Boost::system
)

# HTTP client tests (requires httplib)
find_package(httplib REQUIRED)
add_executable(test_http_api integration/test_http_api_integration.cpp)
target_link_libraries(test_http_api
    axon_recorder
    gtest
    gtest_main
    httplib
)

# Register tests
gtest_discover_tests(test_http_server)
gtest_discover_tests(test_http_api)
```

---

## 11. Implementation Priority

### Phase 1: Infrastructure (P0) - Week 1-2

| Task | Description | Estimate |
|------|-------------|----------|
| Create test directory structure | unit/integration/e2e | 0.5 day |
| Implement test_helpers.hpp | TempDirectory, HttpClient, config factories | 1 day |
| Implement mock plugin | For plugin loading tests | 1 day |
| Configure CMake test builds | Test executables and linking | 1 day |
| Setup CI test pipeline | GitHub Actions configuration | 1 day |

### Phase 2: HTTP API Tests (P0) - Week 3-4

| Task | Description | Estimate |
|------|-------------|----------|
| test_http_server.cpp | All RPC endpoint unit tests (~40 tests) | 3 days |
| test_http_api_integration.cpp | HTTP API integration tests (~30 tests) | 2 days |
| Error handling tests | Invalid requests, state transition errors | 1 day |
| Concurrent request tests | Multi-threaded HTTP client | 1 day |

### Phase 3: Core Component Tests (P0) - Week 5-6

| Task | Description | Estimate |
|------|-------------|----------|
| test_plugin_loader.cpp | Plugin loading tests (~30 tests) | 2 days |
| test_state_machine.cpp | State machine tests (~25 tests) | 1.5 days |
| test_recording_session.cpp | MCAP session tests (~20 tests) | 1.5 days |
| test_worker_thread_pool.cpp | Worker thread pool tests (~15 tests) | 1.5 days |
| test_config_parser.cpp | Config parser tests (~15 tests) | 1 day |

### Phase 4: Integration Tests (P1) - Week 7

| Task | Description | Estimate |
|------|-------------|----------|
| test_recorder_integration.cpp | AxonRecorder integration tests (~25 tests) | 2 days |
| test_plugin_integration.cpp | ROS2 plugin integration tests (~15 tests) | 1.5 days |
| Concurrent integration tests | Race conditions, deadlocks | 1.5 days |

### Phase 5: E2E Tests (P1) - Week 8

| Task | Description | Estimate |
|------|-------------|----------|
| run_e2e_tests.sh | E2E test script | 1 day |
| test_http_api_e2e.sh | HTTP API E2E tests | 1 day |
| MCAP verification tools | File integrity checks | 1 day |
| Performance benchmarks | Throughput, latency | 2 days |

### Phase 6: Stress Tests (P2) - Week 9

| Task | Description | Estimate |
|------|-------------|----------|
| Long-running tests | 24-hour stability | 2 days |
| High-load tests | Maximum throughput | 1 day |
| Resource leak tests | Memory, file descriptors | 1 day |
| Fault injection tests | Disk full, network failure | 1 day |

---

## 12. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-25 | Axon Robotics | Initial version (ROS service architecture) |
| 2.0 | 2025-12-25 | Axon Robotics | Removed mock infrastructure |
| 3.0 | 2026-01-12 | Axon Robotics | **Major refactor**: Plugin architecture + HTTP RPC API |
| | | | - Removed ROS service related tests |
| | | | - Added HTTP RPC API tests (~70 tests) |
| | | | - Added plugin system tests (~30 tests) |
| | | | - Updated component analysis to reflect new architecture |
| | | | - Added E2E test scripts |

---

## 13. Sign-off

| Role | Name | Date | Approval |
|------|------|------|----------|
| Author | | 2026-01-12 | |
| Reviewer (Staff Eng) | | | |
| Reviewer (Test Lead) | | | |

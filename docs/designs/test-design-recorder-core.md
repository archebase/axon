# Test Design Document: Axon Recorder Core Components

**Date:** 2026-01-12
**Document Version:** 3.0

## Executive Summary

本文档提供了 Axon Recorder 的全面测试设计。Axon 是一个高性能的 ROS 录制器，采用**插件化架构**和 **HTTP RPC API** 进行远程控制。

### 架构变化 (v3.0)

与之前的 ROS 服务架构相比，当前实现有以下重大变化：

1. **插件化架构** - 中间件无关的核心 + ROS1/ROS2 插件
2. **HTTP RPC API** - 替代 ROS 服务接口
3. **统一状态机** - IDLE → READY → RECORDING ↔ PAUSED
4. **动态插件加载** - 运行时加载中间件插件

### 核心组件

| 组件 | 文件 | 职责 | 行数 |
|------|------|------|------|
| **AxonRecorder** | [recorder.cpp/hpp](../apps/axon_recorder/recorder.cpp) | 主协调器 | ~350 |
| **HttpServer** | [http_server.cpp/hpp](../apps/axon_recorder/http_server.cpp) | HTTP RPC API | ~570 |
| **PluginLoader** | [plugin_loader.cpp/hpp](../apps/axon_recorder/plugin_loader.cpp) | 插件加载 | ~140 |
| **StateManager** | [state_machine.cpp/hpp](../apps/axon_recorder/state_machine.cpp) | 状态机 | ~200 |
| **RecordingSession** | [recording_session.cpp/hpp](../apps/axon_recorder/recording_session.cpp) | MCAP 会话 | ~200 |
| **WorkerThreadPool** | [worker_thread_pool.cpp/hpp](../apps/axon_recorder/worker_thread_pool.cpp) | 工作线程池 | ~270 |
| **ConfigParser** | [config_parser.cpp/hpp](../apps/axon_recorder/config_parser.cpp) | 配置解析 | ~370 |
| **MetadataInjector** | [metadata_injector.cpp/hpp](../apps/axon_recorder/metadata_injector.cpp) | 元数据注入 | ~400 |
| **HttpCallbackClient** | [http_callback_client.cpp/hpp](../apps/axon_recorder/http_callback_client.cpp) | HTTP 回调 | ~360 |

### 测试目标

测试套件需要从零开始设计，因为：

1. **新的 HTTP RPC API** - 需要完整的 HTTP 端点测试
2. **插件系统** - 需要插件加载和 ABI 兼容性测试
3. **状态机** - 需要完整的状态转换测试
4. **并发模型** - 需要线程安全和竞态条件测试

目标：**~240 个测试**，覆盖率 **>85%**

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

### 1.1 系统架构

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

### 1.2 HTTP RPC 端点

| Method | Endpoint | 状态转换 | 说明 |
|--------|----------|---------|------|
| POST | `/rpc/config` | IDLE → READY | 设置任务配置 |
| POST | `/rpc/begin` | READY → RECORDING | 开始录制 |
| POST | `/rpc/finish` | RECORDING/PAUSED → IDLE | 完成录制 |
| POST | `/rpc/pause` | RECORDING → PAUSED | 暂停录制 |
| POST | `/rpc/resume` | PAUSED → RECORDING | 恢复录制 |
| POST | `/rpc/cancel` | RECORDING/PAUSED → IDLE | 取消录制 |
| POST | `/rpc/clear` | READY → IDLE | 清除配置 |
| POST | `/rpc/quit` | Any → Exit | 退出程序 |
| GET | `/rpc/state` | - | 获取当前状态 |
| GET | `/rpc/stats` | - | 获取统计信息 |
| GET | `/` or `/health` | - | 健康检查 |

详细 API 规范见 [rpc-api-design.md](./rpc-api-design.md)。

### 1.3 插件 ABI 接口

插件通过以下 C 结构体暴露功能：

```cpp
// 插件描述符
struct AxonPluginDescriptor {
  uint32_t abi_version_major;
  uint32_t abi_version_minor;
  const char* middleware_name;
  const char* middleware_version;
  const char* plugin_version;
  AxonPluginVtable* vtable;
  void* reserved[16];
};

// 插件函数表
struct AxonPluginVtable {
  AxonInitFn init;           // 初始化插件
  AxonStartFn start;         // 启动插件
  AxonStopFn stop;           // 停止插件
  AxonSubscribeFn subscribe; // 订阅主题
  AxonPublishFn publish;     // 发布消息
  void* reserved[9];
};
```

---

## 2. Component Analysis

### 2.1 AxonRecorder (recorder.cpp)

**职责：**
- 插件加载和生命周期管理
- HTTP RPC 服务器管理
- 状态机协调
- 工作线程池管理
- MCAP 会话管理
- 元数据注入

**关键方法：**

| 方法 | 复杂度 | 测试优先级 |
|------|--------|-----------|
| `initialize()` | 高 | P0 |
| `start()` | 高 | P0 |
| `stop()` | 高 | P0 |
| `set_task_config()` | 低 | P1 |
| `on_message()` | 中 | P0 |
| `start_http_server()` | 中 | P0 |
| `message_handler()` | 中 | P1 |
| `register_topics()` | 中 | P1 |

**测试覆盖重点：**
- 初始化失败场景（插件不存在、配置错误）
- 状态转换的正确性
- 并发消息处理
- 资源清理（析构、stop）

### 2.2 HttpServer (http_server.cpp)

**职责：**
- HTTP 请求处理
- JSON 请求/响应解析
- RPC 端点路由
- 错误处理

**关键方法：**

| 方法 | 复杂度 | 测试优先级 |
|------|--------|-----------|
| `handle_request()` | 高 | P0 |
| `handle_rpc_begin()` | 中 | P0 |
| `handle_rpc_finish()` | 中 | P0 |
| `handle_rpc_set_config()` | 中 | P0 |
| `handle_rpc_get_state()` | 低 | P0 |
| `handle_rpc_get_stats()` | 低 | P1 |
| `handle_rpc_pause()` | 低 | P2 |
| `handle_rpc_resume()` | 低 | P2 |
| `handle_rpc_cancel()` | 低 | P2 |
| `handle_rpc_clear()` | 低 | P2 |
| `handle_rpc_quit()` | 高 | P0 |

**测试覆盖重点：**
- 所有 RPC 端点的正常流程
- 状态转换错误（从错误状态调用）
- 无效 JSON 请求
- 并发 HTTP 请求
- 服务器启动/停止

### 2.3 PluginLoader (plugin_loader.cpp)

**职责：**
- 动态库加载（dlopen）
- 符号解析（dlsym）
- 插件生命周期管理
- ABI 版本验证

**关键方法：**

| 方法 | 复杂度 | 测试优先级 |
|------|--------|-----------|
| `load()` | 高 | P0 |
| `unload()` | 中 | P0 |
| `get_descriptor()` | 低 | P0 |
| `get_plugin()` | 低 | P1 |

**测试覆盖重点：**
- 成功加载有效插件
- 加载不存在的库
- 加载无效插件（缺少符号）
- ABI 版本不匹配
- 重复加载
- 卸载和资源清理

### 2.4 StateManager (state_machine.cpp)

**职责：**
- 状态转换管理
- 转换验证
- 转换回调

**状态转换图：**
```
IDLE → READY → RECORDING ↔ PAUSED
  ↑                         ↓
  └───────── (finish/cancel)
```

**测试覆盖重点：**
- 所有效状态转换
- 无效状态转换
- 转换回调触发
- 并发转换尝试

### 2.5 RecordingSession (recording_session.cpp)

**职责：**
- MCAP 文件打开/关闭
- Schema 和通道注册
- 消息写入
- 统计信息收集

**测试覆盖重点：**
- 文件生命周期
- Schema 注册失败
- 重复注册处理
- 并发写入
- 元数据注入

### 2.6 WorkerThreadPool (worker_thread_pool.cpp)

**职责：**
- 工作线程创建/销毁
- SPSC 队列管理
- 消息分发

**测试覆盖重点：**
- 线程创建/销毁
- 队列满时的行为
- 消息顺序
- 优雅关闭

---

## 3. Test Strategy

### 3.1 测试金字塔

```
                    ╭─────────────────╮
                    │   E2E Tests     │  ← HTTP API 完整流程
                    │   (~20 tests)   │
                ╭───┴─────────────────┴───╮
                │  Integration Tests      │  ← 组件集成
                │  (~60 tests)            │
            ╭───┴─────────────────────────┴───╮
            │        Unit Tests               │  ← 单个组件
            │        (~150 tests)            │
        ╭───┴─────────────────────────────────┴───╮
        │     Core Libraries (axon_*)             │
        │     (独立测试，已存在)                    │
        ╰─────────────────────────────────────────╯
```

### 3.2 测试分类

| 类别 | 范围 | 框架 | ROS 要求 | 典型运行时间 |
|------|------|------|----------|-------------|
| **Unit** | 单个组件，隔离依赖 | GTest | 否 | <10ms |
| **Integration** | 多组件协作 | GTest | 否* | 50-200ms |
| **HTTP API** | HTTP 端点测试 | GTest + httplib | 否 | 20-100ms |
| **E2E** | 完整录制流程 | Shell/Python | 是* | 1-10s |
| **Stress** | 高并发、长时间 | GTest | 否 | 5-60s |
| **Performance** | 吞吐量、延迟 | Custom | 是 | 1-5min |

*注：Integration 测试可以通过模拟插件避免 ROS 依赖；E2E 测试需要真实的 ROS 环境。

### 3.3 测试覆盖目标

| 组件 | 行覆盖 | 分支覆盖 |
|------|--------|----------|
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

### 4.1 测试文件结构

```
apps/axon_recorder/test/
├── CMakeLists.txt
├── unit/
│   ├── test_helpers.hpp
│   ├── test_http_server.cpp           # HTTP 服务器单元测试 (~40 tests)
│   ├── test_plugin_loader.cpp         # 插件加载器测试 (~30 tests)
│   ├── test_state_machine.cpp         # 状态机测试 (~25 tests)
│   ├── test_recording_session.cpp     # MCAP 会话测试 (~20 tests)
│   ├── test_worker_thread_pool.cpp    # 工作线程池测试 (~15 tests)
│   ├── test_config_parser.cpp         # 配置解析测试 (~15 tests)
│   └── test_task_config.cpp           # 任务配置测试 (~10 tests)
├── integration/
│   ├── test_recorder_integration.cpp  # AxonRecorder 集成测试 (~25 tests)
│   ├── test_http_api_integration.cpp  # HTTP API 集成测试 (~30 tests)
│   └── test_plugin_integration.cpp    # 插件集成测试 (~15 tests)
└── e2e/
    ├── run_e2e_tests.sh               # E2E 测试脚本
    └── test_http_api_e2e.sh           # HTTP API E2E 测试
```

### 4.2 HTTP 服务器单元测试 (test_http_server.cpp)

**测试夹具：**
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

**测试用例：**

#### 4.2.1 服务器生命周期 (5 tests)

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

#### 4.2.13 错误处理 (8 tests)

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

### 4.3 HTTP 集成测试 (test_http_api_integration.cpp)

**测试完整流程：**

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

### 5.1 插件加载器测试 (test_plugin_loader.cpp)

**测试夹具：**
```cpp
class PluginLoaderTest : public ::testing::Test {
protected:
  void SetUp() override;
  void TearDown() override;

  std::string test_plugin_dir_;
  axon::recorder::PluginLoader loader_;
};
```

**测试用例：**

#### 5.1.1 加载有效插件 (8 tests)

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

#### 5.1.2 加载无效插件 (10 tests)

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

#### 5.1.3 卸载插件 (6 tests)

```cpp
TEST_F(PluginLoaderTest, Unload_LoadedPlugin_Succeeds)
TEST_F(PluginLoaderTest, Unload_ByName_Succeeds)
TEST_F(PluginLoaderTest, Unload_NonExistentPlugin_ReturnsFalse)
TEST_F(PluginLoaderTest, Unload_All_Succeeds)
TEST_F(PluginLoaderTest, Unload_CallsStopIfRunning)
TEST_F(PluginLoaderTest, Unload_ClosesHandle)
```

#### 5.1.4 查询插件 (6 tests)

```cpp
TEST_F(PluginLoaderTest, GetDescriptor_LoadedPlugin_ReturnsDescriptor)
TEST_F(PluginLoaderTest, GetDescriptor_NonExistentPlugin_ReturnsNull)
TEST_F(PluginLoaderTest, GetPlugin_LoadedPlugin_ReturnsPlugin)
TEST_F(PluginLoaderTest, GetPlugin_NonExistentPlugin_ReturnsNull)
TEST_F(PluginLoaderTest, IsLoaded_LoadedPlugin_ReturnsTrue)
TEST_F(PluginLoaderTest, LoadedPlugins_ReturnsListOfNames)
```

### 5.2 模拟插件

**测试用的模拟插件实现：**

```cpp
// test/mock_plugin.cpp
extern "C" {

// 模拟插件描述符
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

### 6.1 状态机测试 (test_state_machine.cpp)

**测试用例：**

#### 6.1.1 有效转换 (10 tests)

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

#### 6.1.2 无效转换 (10 tests)

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

#### 6.1.3 转换回调 (5 tests)

```cpp
TEST(StateMachineTest, Transition_CallsCallback)
TEST(StateMachineTest, Transition_WithMultipleCallbacks_CallsAll)
TEST(StateMachineTest, Transition_Failed_DoesNotCallCallback)
TEST(StateMachineTest, Callback_ReceivesFromAndToStates)
TEST(StateMachineTest, CanUnregisterCallback)
```

#### 6.1.4 状态查询 (5 tests)

```cpp
TEST(StateMachineTest, GetState_ReturnsInitialState)
TEST(StateMachineTest, GetState_AfterTransition_ReturnsNewState)
TEST(StateMachineTest, GetStateString_ReturnsCorrectString)
TEST(StateMachineTest, IsRecording_WhenRecording_ReturnsTrue)
TEST(StateMachineTest, IsRecording_WhenNotRecording_ReturnsFalse)
```

---

## 7. Concurrency Tests

### 7.1 并发测试场景

| 场景 | 描述 | 测试方法 |
|------|------|----------|
| **并发 HTTP 请求** | 多个客户端同时调用 API | 多线程 HTTP 客户端 |
| **并发消息写入** | 多个主题同时写入 | 多线程发布者 |
| **状态转换竞态** | 同时尝试状态转换 | 多线程调用转换 |
| **队列溢出** | 快速发布者填满队列 | 高频消息生成 |
| **优雅关闭** | 录制时关闭服务器 | Stop + 资源检查 |

### 7.2 并发测试用例

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

### 7.3 压力测试

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

### 8.1 AxonRecorder 集成测试 (test_recorder_integration.cpp)

**测试完整流程：**

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

### 8.2 插件集成测试 (test_plugin_integration.cpp)

**测试与 ROS2 插件的集成：**

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

### 9.1 E2E 测试脚本 (test_http_api_e2e.sh)

```bash
#!/bin/bash
# E2E 测试脚本 - 完整录制流程

set -e

AXON_RECORDER_BIN="./build/axon_recorder"
PLUGIN_PATH="./middlewares/ros2/src/ros2_plugin/install/libaxon_ros2_plugin.so"
CONFIG_FILE="./test/e2e/test_config.yaml"

# 启动 recorder
$AXON_RECORDER_BIN --plugin $PLUGIN_PATH --config $CONFIG_FILE &
RECORDER_PID=$!

# 等待启动
sleep 2

# 1. 健康检查
curl -f http://localhost:8080/health || exit 1

# 2. 设置配置
curl -X POST http://localhost:8080/rpc/config \
  -H "Content-Type: application/json" \
  -d '{
    "task_config": {
      "task_id": "e2e_test_001",
      "device_id": "test_robot",
      "topics": ["/test/topic1", "/test/topic2"]
    }
  }' || exit 1

# 3. 检查状态 (应该是 READY)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "ready" ] || exit 1

# 4. 开始录制
curl -X POST http://localhost:8080/rpc/begin || exit 1

# 5. 检查状态 (应该是 RECORDING)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "recording" ] || exit 1

# 6. 发布测试消息
# (使用 ROS2 发布测试数据)

# 7. 等待录制
sleep 5

# 8. 检查统计信息
curl -s http://localhost:8080/rpc/stats || exit 1

# 9. 完成录制
curl -X POST http://localhost:8080/rpc/finish \
  -H "Content-Type: application/json" \
  -d '{"task_id": "e2e_test_001"}' || exit 1

# 10. 检查状态 (应该是 IDLE)
STATE=$(curl -s http://localhost:8080/rpc/state | jq -r '.data.state')
[ "$STATE" == "idle" ] || exit 1

# 11. 验证 MCAP 文件
[ -f "output.mcap" ] || exit 1

# 12. 验证 sidecar 文件
[ -f "output.mcap.json" ] || exit 1

# 清理
kill $RECORDER_PID
wait $RECORDER_PID

echo "E2E tests passed!"
```

### 9.2 E2E 测试场景

| 场景 | 描述 | 验证点 |
|------|------|--------|
| **基本流程** | config → begin → finish | MCAP 文件生成 |
| **暂停恢复** | begin → pause → resume → finish | 数据完整性 |
| **取消录制** | begin → cancel | 无 sidecar 生成 |
| **清除配置** | config → clear | 配置已清除 |
| **多任务** | 多次 begin/finish | 每次独立文件 |
| **错误恢复** | 无效请求处理 | 正确错误响应 |

---

## 10. Test Infrastructure

### 10.1 测试支持库

**test_helpers.hpp:**

```cpp
namespace axon::recorder::test {

// 测试配置
TaskConfig make_test_task_config();
RecorderConfig make_test_recorder_config();

// 临时目录管理
class TempDirectory {
public:
  TempDirectory();
  ~TempDirectory();
  std::filesystem::path path() const;
  std::filesystem::path file(const std::string& name) const;
};

// HTTP 测试客户端
class HttpClient {
public:
  HttpClient(const std::string& base_url);
  nlohmann::json post(const std::string& endpoint, const nlohmann::json& body);
  nlohmann::json get(const std::string& endpoint);
};

// MCAP 验证
bool verify_mcap_file(const std::string& path);
bool verify_sidecar_file(const std::string& path);

}  // namespace axon::recorder::test
```

### 10.2 CMake 配置

**test/CMakeLists.txt:**

```cmake
# 测试可执行文件
add_executable(test_http_server unit/test_http_server.cpp)
target_link_libraries(test_http_server
    axon_recorder
    gtest
    gtest_main
    Boost::system
)

# HTTP 客户端测试（需要 httplib）
find_package(httplib REQUIRED)
add_executable(test_http_api integration/test_http_api_integration.cpp)
target_link_libraries(test_http_api
    axon_recorder
    gtest
    gtest_main
    httplib
)

# 注册测试
gtest_discover_tests(test_http_server)
gtest_discover_tests(test_http_api)
```

---

## 11. Implementation Priority

### Phase 1: 基础设施 (P0) - 第1-2周

| 任务 | 描述 | 估算 |
|------|------|------|
| 创建测试目录结构 | unit/integration/e2e | 0.5天 |
| 实现 test_helpers.hpp | TempDirectory, HttpClient, 配置工厂 | 1天 |
| 实现 mock plugin | 用于插件加载测试 | 1天 |
| 配置 CMake 测试构建 | 测试可执行文件和链接 | 1天 |
| 设置 CI 测试流水线 | GitHub Actions 配置 | 1天 |

### Phase 2: HTTP API 测试 (P0) - 第3-4周

| 任务 | 描述 | 估算 |
|------|------|------|
| test_http_server.cpp | 所有 RPC 端点单元测试 (~40 tests) | 3天 |
| test_http_api_integration.cpp | HTTP API 集成测试 (~30 tests) | 2天 |
| 错误处理测试 | 无效请求、状态转换错误 | 1天 |
| 并发请求测试 | 多线程 HTTP 客户端 | 1天 |

### Phase 3: 核心组件测试 (P0) - 第5-6周

| 任务 | 描述 | 估算 |
|------|------|------|
| test_plugin_loader.cpp | 插件加载测试 (~30 tests) | 2天 |
| test_state_machine.cpp | 状态机测试 (~25 tests) | 1.5天 |
| test_recording_session.cpp | MCAP 会话测试 (~20 tests) | 1.5天 |
| test_worker_thread_pool.cpp | 工作线程池测试 (~15 tests) | 1.5天 |
| test_config_parser.cpp | 配置解析测试 (~15 tests) | 1天 |

### Phase 4: 集成测试 (P1) - 第7周

| 任务 | 描述 | 估算 |
|------|------|------|
| test_recorder_integration.cpp | AxonRecorder 集成测试 (~25 tests) | 2天 |
| test_plugin_integration.cpp | ROS2 插件集成测试 (~15 tests) | 1.5天 |
| 并发集成测试 | 竞态条件、死锁 | 1.5天 |

### Phase 5: E2E 测试 (P1) - 第8周

| 任务 | 描述 | 估算 |
|------|------|------|
| run_e2e_tests.sh | E2E 测试脚本 | 1天 |
| test_http_api_e2e.sh | HTTP API E2E 测试 | 1天 |
| MCAP 验证工具 | 文件完整性检查 | 1天 |
| 性能基准测试 | 吞吐量、延迟 | 2天 |

### Phase 6: 压力测试 (P2) - 第9周

| 任务 | 描述 | 估算 |
|------|------|------|
| 长时间运行测试 | 24小时稳定性 | 2天 |
| 高负载测试 | 最大吞吐量 | 1天 |
| 资源泄漏测试 | 内存、文件描述符 | 1天 |
| 故障注入测试 | 磁盘满、网络故障 | 1天 |

---

## 12. Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-25 | Axon Robotics | 初始版本 (ROS 服务架构) |
| 2.0 | 2025-12-25 | Axon Robotics | 移除 mock 基础设施 |
| 3.0 | 2026-01-12 | Axon Robotics | **重大重构**: 插件架构 + HTTP RPC API |
| | | | - 移除 ROS 服务相关测试 |
| | | | - 添加 HTTP RPC API 测试 (~70 tests) |
| | | | - 添加插件系统测试 (~30 tests) |
| | | | - 更新组件分析以反映新架构 |
| | | | - 添加 E2E 测试脚本 |

---

## 13. Sign-off

| Role | Name | Date | Approval |
|------|------|------|----------|
| Author | | 2026-01-12 | |
| Reviewer (Staff Eng) | | | |
| Reviewer (Test Lead) | | | |

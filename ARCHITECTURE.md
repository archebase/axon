# Axon Architecture

## Overview

Axon is a high-performance ROS recorder by ArcheBase that writes data to MCAP format. It supports both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling) with a task-centric design for fleet management via ros-bridge.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Server / Fleet Manager                          │
│                              (via ros-bridge)                                │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    ▼                 ▼                 ▼
              CachedRecording   RecordingControl   RecordingStatus
                  Config            Service            Service
                    │                 │                 │
                    └─────────────────┼─────────────────┘
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           axon_recorder (ROS Node)                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │  State Machine  │  │ Recording       │  │ HTTP Callback Client        │  │
│  │  (4-state FSM)  │  │ Service Impl    │  │ (start/finish notify)       │  │
│  └────────┬────────┘  └────────┬────────┘  └─────────────────────────────┘  │
│           │                    │                                             │
│           ▼                    ▼                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                     Recording Session                                │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐  │    │
│  │  │ Worker Thread   │  │ MCAP Writer     │  │ Metadata Injector   │  │    │
│  │  │ Pool (SPSC)     │  │ Wrapper         │  │ + Sidecar JSON      │  │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           C++ Core Libraries                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────────────┐  │
│  │ axon_mcap       │  │ axon_logging    │  │ axon_uploader               │  │
│  │ (MCAP writer)   │  │ (Boost.Log)     │  │ (S3 upload + retry)         │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## State Machine

The recorder uses a task-centric state machine where each task corresponds to one MCAP file.

```
        ┌─────────────────────────────────────────────────────┐
        │                                                     │
        ▼                                                     │
   ┌─────────┐                                                │
   │  IDLE   │ ◄──────────────────────────────────────────────┤
   └────┬────┘                                                │
        │ CachedRecordingConfig                               │
        ▼                                                     │
   ┌─────────┐                                                │
   │  READY  │────────(clear/timeout)─────────────────────────┤
   └────┬────┘                                                │
        │ start                                               │
        ▼                                                     │
   ┌─────────┐  pause   ┌─────────┐                           │
   │RECORDING│─────────►│ PAUSED  │                           │
   │         │◄─────────│         │                           │
   └────┬────┘  resume  └────┬────┘                           │
        │                    │                                │
        └────────┬───────────┘                                │
                 │                                            │
         ┌───────┴───────┐                                    │
      finish           cancel                                 │
         │               │                                    │
         ▼               ▼                                    │
   (finalize MCAP)  (cleanup)                                 │
         │               │                                    │
         └───────────────┴────────────────────────────────────┘
```

| State | Description |
|-------|-------------|
| `IDLE` | No active task, waiting for configuration |
| `READY` | Task config cached, ready to start recording |
| `RECORDING` | Actively recording ROS messages to MCAP |
| `PAUSED` | Recording paused, can resume or finish |

## Core Components

### 1. ROS Recorder (`middlewares/src/axon_recorder/`)

**Recording Session**
- Encapsulates MCAP file lifecycle (open/write/close)
- Thread-safe message writing with schema/channel registration
- Tracks session statistics (messages, bytes, duration)

**Worker Thread Pool**
- Per-topic lock-free SPSC queues for zero-copy message transfer
- Configurable queue capacity and idle backoff
- Pause/resume support for recording state

**State Manager**
- Thread-safe state transitions with validation
- StateTransactionGuard for RAII rollback on failure
- Transition callbacks for observers

**Metadata Injector**
- Injects `axon.task`, `axon.device`, `axon.recording` metadata into MCAP
- Generates sidecar JSON with SHA-256 checksum
- Cascading config resolution (env → config → ROS params)

**HTTP Callback Client**
- POST notifications to server on start/finish
- JWT Bearer token authentication
- SSL/TLS support for HTTPS

### 2. MCAP Writer (`core/axon_mcap/`)

- Thread-safe MCAP file operations
- Zstd/LZ4 compression support
- MCAP validator for file integrity

### 3. Logging Infrastructure (`core/axon_logging/`)

- Boost.Log based with async sinks
- Console sink (colors, severity filtering)
- File sink (rotation, JSON/text format)
- ROS sink adapter for RCLCPP/ROSCPP integration
- Environment variable configuration

### 4. Edge Uploader (`core/axon_uploader/`)

- S3 multipart upload for large files
- SQLite state persistence for crash recovery
- Exponential backoff retry with jitter
- MCAP-first, JSON-last upload order (JSON signals completion)
- Backpressure alerts when queue grows

## Service API

| Service | Purpose |
|---------|---------|
| `CachedRecordingConfig` | Cache task configuration from server |
| `IsRecordingReady` | Query if recorder has cached config |
| `RecordingControl` | Unified control: start/pause/resume/cancel/finish/clear |
| `RecordingStatus` | Query status, metrics, and task info |

## Data Flow

```
ROS Topics
    │
    ▼ (subscription callbacks)
┌─────────────────────────────────────────┐
│ Serialization (ROS1/ROS2 format)        │
└─────────────────────────────────────────┘
    │
    ▼ (zero-copy via SPSC queue)
┌─────────────────────────────────────────┐
│ Worker Thread Pool                      │
│ - Per-topic queues                      │
│ - Sequence numbering                    │
└─────────────────────────────────────────┘
    │
    ▼ (thread-safe write)
┌─────────────────────────────────────────┐
│ MCAP Writer                             │
│ - Schema/channel registration           │
│ - Compression (Zstd/LZ4)                │
└─────────────────────────────────────────┘
    │
    ▼ (on finish)
┌─────────────────────────────────────────┐
│ Finalization                            │
│ - Inject metadata records               │
│ - Generate sidecar JSON + checksum      │
│ - HTTP callback to server               │
└─────────────────────────────────────────┘
    │
    ▼ (async)
┌─────────────────────────────────────────┐
│ Edge Uploader → S3                      │
└─────────────────────────────────────────┘
```

## Threading Model

| Thread | Responsibility |
|--------|----------------|
| ROS Executor | Service callbacks, subscription callbacks |
| Worker Threads | Drain SPSC queues, write to MCAP (one per topic) |
| HTTP Client | Async callbacks to server |
| Uploader Workers | S3 upload with retry |

## Lock-Free Design

- **SPSC Queues**: Cache-line aligned, acquire-release semantics
- **MPSC Queue**: Multiple producers round-robin to dedicated SPSC queues
- **Atomic Statistics**: Lock-free counters for monitoring

## Middleware Integration Architecture

Axon uses a plugin-based middleware integration architecture that cleanly separates middleware-specific code from core functionality.

### Design Principles

1. **Middleware Isolation**: Each middleware (ROS1, ROS2, etc.) resides in `middlewares/{name}/` and compiles independently into a dynamic library
2. **Core Independence**: All components outside `middlewares/` are middleware-agnostic with no ROS or middleware dependencies
3. **Unified C API**: Middlewares expose a unified C API interface for integration with the main application
4. **Dynamic Plugin Loading**: The main program loads middleware plugins as dynamic libraries at runtime

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Main Application                                │
│                    (Middleware-Agnostic Core)                          │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                 Plugin Loader (dlopen/dlsym)                     │  │
│  └────────────────────┬────────────────────┬────────────────────────┘  │
│                       │                    │                             │
│  ┌────────────────────▼────────────────────▼────────────────────────┐  │
│  │              Unified C API Interface                             │  │
│  │         (middleware_abi.h / plugin.h)                            │  │
│  └────────────────────┬────────────────────┬────────────────────────┘  │
│                       │                    │                             │
│           ┌───────────▼──────┐   ┌───────▼──────────┐                   │
│           │  ROS1 Plugin     │   │  ROS2 Plugin     │                   │
│           │  (.so)           │   │  (.so)           │                   │
│           └──────────────────┘   └──────────────────┘                   │
└─────────────────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         Core Libraries                                  │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────────────────┐  │
│  │ axon_mcap     │  │ axon_logging  │  │ axon_uploader              │  │
│  │ (no ROS deps) │  │ (no ROS deps) │  │ (no ROS deps)              │  │
│  └───────────────┘  └───────────────┘  └───────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

### Directory Structure

```
Axon/
├── core/                      # Middleware-agnostic core libraries
│   ├── axon_mcap/            # MCAP writer (no ROS dependencies)
│   ├── axon_logging/         # Logging infrastructure (no ROS dependencies)
│   └── axon_uploader/        # S3 uploader (no ROS dependencies)
│
├── middlewares/              # Middleware-specific plugins
│   ├── ros1/                 # ROS1 (Noetic) plugin
│   │   ├── include/          # Public C API headers
│   │   ├── src/              # Implementation
│   │   └── CMakeLists.txt    # Builds libaxon_ros1.so
│   │
│   └── ros2/                 # ROS2 (Humble/Jazzy/Rolling) plugin
│       ├── include/          # Public C API headers
│       ├── src/              # Implementation
│       └── CMakeLists.txt    # Builds libaxon_ros2.so
│
└── include/                  # Unified plugin interface
    └── middleware_abi.h      # C API definitions for plugin communication
```

### Plugin Interface

Each middleware plugin implements a standardized C API:

```c
// Plugin lifecycle
typedef struct {
    int (*init)(const char* config);
    int (*start)(void);
    int (*stop)(void);
    void (*shutdown)(void);
} middleware_plugin_t;

// Plugin registration
typedef struct {
    const char* name;
    const char* version;
    middleware_plugin_t* vtable;
} middleware_descriptor_t;

// Exported symbol
extern const middleware_descriptor_t* middleware_get_descriptor(void);
```

### Benefits

1. **Clean Separation**: Core libraries have zero middleware dependencies
2. **Extensibility**: New middlewares can be added without touching core code
3. **Testing**: Core libraries can be tested independently of ROS
4. **Deployment**: Only required middleware plugins need to be deployed
5. **Maintenance**: Middleware-specific bugs are isolated to plugin code

## Performance Characteristics

- **Zero-copy**: Messages moved through queue without copying
- **Direct serialization**: No schema conversion overhead
- **Bounded memory**: Fixed-capacity queues with backpressure
- **Async I/O**: Non-blocking HTTP callbacks and uploads
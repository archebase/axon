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
│                    Axon Recorder (Unified Plugin System)                     │
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
                    ┌─────────────────┴─────────────────┐
                    ▼                                   ▼
┌───────────────────────────────┐       ┌───────────────────────────────────────┐
│   Middleware Plugins          │       │         C++ Core Libraries           │
│  ┌─────────────────────────┐  │       │  ┌─────────────────┐  ┌─────────────┐ │
│  │ middlewares/ros2/      │  │       │  │ axon_mcap       │  │axon_logging │ │
│  │ - libros2_plugin.so    │  │       │  │ (MCAP writer)   │  │ (Boost.Log) │ │
│  │ - rclcpp::GenericSub   │  │       │  └─────────────────┘  └─────────────┘ │
│  └─────────────────────────┘  │       │  ┌─────────────────────────────────┐ │
│  ┌─────────────────────────┐  │       │  │ axon_uploader                   │ │
│  │ middlewares/ros1/      │  │       │  │ (S3 upload + retry)             │ │
│  │ - libros1_plugin.so    │  │       │  └─────────────────────────────────┘ │
│  │ - ShapeShifter         │  │       └───────────────────────────────────────┘
│  └─────────────────────────┘  │
└───────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Unified Test Programs (examples/)                        │
│  ┌─────────────────────────┐  ┌─────────────────────────────────────────┐  │
│  │ subscriber_test         │  │ plugin_interface.hpp (NO ROS deps!)     │  │
│  │ - Zero compile-time deps│  │ - Unified API for ROS1 & ROS2           │  │
│  │ - Dynamic loading       │  │ - Factory functions & callbacks          │  │
│  └─────────────────────────┘  └─────────────────────────────────────────┘  │
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

### 1. ROS Recorder (`middlewares/ros2/` and `middlewares/ros1/`)

The Axon recorder is organized into separate middleware implementations under the `middlewares/` directory:

**ROS2 Implementation** (`middlewares/ros2/`)
- Standard ROS2 workspace with colcon build system
- Universal subscriber plugin using `rclcpp::GenericSubscription`
- Supports any ROS2 message type without compile-time dependencies
- Dynamic plugin loading via `libros2_plugin.so`

**ROS1 Implementation** (`middlewares/ros1/`)
- Standard ROS1 workspace with catkin build system
- Universal subscriber plugin using `topic_tools::ShapeShifter`
- Supports any ROS1 message type without compile-time dependencies
- Dynamic plugin loading via `libros1_plugin.so`

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

### 5. Unified Test Programs (`examples/`)

- **Zero compile-time ROS dependencies** - only needs `libdl`
- Dynamic plugin loading for both ROS1 and ROS2
- Single binary works with any message type
- Unified interface via `plugin_interface.hpp`

## Service API

| Service | Purpose |
|---------|---------|
| `CachedRecordingConfig` | Cache task configuration from server |
| `IsRecordingReady` | Query if recorder has cached config |
| `RecordingControl` | Unified control: start/pause/resume/cancel/finish/clear |
| `RecordingStatus` | Query status, metrics, and task info |

## Data Flow

```
ROS Topics (ROS1 or ROS2)
    │
    ▼ (subscription callbacks)
┌─────────────────────────────────────────┐
│ Middleware Plugins                      │
│ - middlewares/ros2/ (rclcpp::GenericSubscription) │
│ - middlewares/ros1/ (topic_tools::ShapeShifter)   │
│ - Raw serialized message data          │
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
│ MCAP Writer (core/axon_mcap/)            │
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
│ Edge Uploader (core/axon_uploader/) → S3 │
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

## Performance Characteristics

- **Zero-copy**: Messages moved through queue without copying
- **Direct serialization**: No schema conversion overhead
- **Bounded memory**: Fixed-capacity queues with backpressure
- **Async I/O**: Non-blocking HTTP callbacks and uploads

## Directory Structure

```
axon/
├── middlewares/              # Middleware implementations
│   ├── ros2/                 # ROS2 plugin workspace
│   │   ├── src/ros2_plugin/  # ROS2 universal subscriber
│   │   ├── build/            # Colcon build output (gitignored)
│   │   ├── install/          # Colcon install output (gitignored)
│   │   └── build.sh          # ROS2 build script
│   └── ros1/                 # ROS1 plugin workspace
│       ├── src/ros1_plugin/  # ROS1 universal subscriber
│       ├── build/            # Catkin build output (gitignored)
│       ├── devel/            # Catkin devel space (gitignored)
│       ├── build.sh          # ROS1 build script
│       └── clean.sh          # ROS1 clean script
├── core/                     # Core C++ libraries
│   ├── axon_mcap/            # MCAP writer wrapper
│   ├── axon_uploader/        # Edge upload functionality
│   ├── axon_logging/         # Logging utilities
│   └── Makefile              # C++ libraries build system
├── examples/                 # Unified test programs
│   ├── plugin_interface.hpp  # Unified plugin interface (NO ROS deps)
│   ├── subscriber_test_unified.cpp  # Unified test implementation
│   ├── CMakeLists.txt        # Build config (only needs libdl)
│   ├── build.sh              # Build script
│   ├── run_ros2.sh           # Run script for ROS2
│   ├── run_ros1.sh           # Run script for ROS1
│   ├── test_unified.sh       # Test script
│   └── build/                # Build output (gitignored)
│       └── subscriber_test   # Unified binary (works with both!)
├── scripts/                  # Build and clean scripts
│   ├── build.sh              # Root build script
│   └── clean.sh              # Root clean script
├── Makefile                  # Unified build system (recommended)
├── .gitignore                # Git ignore rules
├── ARCH.md                   # Original project requirements
├── ARCHITECTURE.md           # This file - architecture documentation
├── CLAUDE.md                 # Complete technical documentation
├── MCAP_INTEGRATION.md       # MCAP integration guide
└── README.md                 # Project overview and quick start
```

## Build System

The Axon project uses a unified Makefile that provides a single entry point for all build operations:

### Primary Build Targets

- `make build` - Build ROS2 plugin and unified test program
- `make build-all` - Build everything (ROS1 + ROS2 + examples)
- `make build-ros2` - Build ROS2 plugin only
- `make build-ros1` - Build ROS1 plugin only
- `make build-examples` - Build unified test program (no ROS deps)

### Run Targets

- `make run-ros2` - Run unified test with ROS2 plugin
- `make run-ros1` - Run unified test with ROS1 plugin

### Clean Targets

- `make clean` - Clean all build artifacts
- `make clean-ros2` - Clean ROS2 build artifacts
- `make clean-ros1` - Clean ROS1 build artifacts
- `make clean-examples` - Clean examples build artifacts

### Other Targets

- `make help` - Show all available commands
- `make info` - Show build information and ROS installation status
- `make install` - Set executable permissions on all scripts

## Key Design Principles

### 1. Middleware Abstraction

The `middlewares/` directory separates ROS-specific implementations from the core application logic:

- **Standard ROS workspaces**: Each middleware uses its standard build system (colcon for ROS2, catkin for ROS1)
- **Dynamic plugin loading**: Plugins are loaded at runtime via `dlopen()`, eliminating compile-time dependencies
- **Universal subscription**: Both plugins support any message type without prior knowledge

### 2. Zero Compile-Time Dependencies

The unified test program in `examples/` demonstrates the power of this architecture:

- **No ROS headers needed**: Compiles with only `libdl`
- **Single binary**: Works with both ROS1 and ROS2 via command-line selection
- **Type-agnostic**: Can subscribe to any message type specified as a string

### 3. Core Library Separation

The `core/` directory contains reusable C++ libraries that are middleware-agnostic:

- **axon_mcap**: MCAP file operations with compression
- **axon_logging**: Boost.Log-based logging with async sinks
- **axon_uploader**: S3 multipart upload with retry logic

### 4. Unified Interface

The `plugin_interface.hpp` defines a common API for both ROS1 and ROS2:

```cpp
// Common callback type for both ROS1 and ROS2
typedef void (*RawDataCallback)(const uint8_t* data, size_t size,
                                 uint64_t timestamp, const char* topic,
                                 const char* msg_type);

// Plugin factory functions (extern "C")
void init_ros2_context();
void* create_subscriber(const char* topic, const char* type);
void set_data_callback(void* sub, RawDataCallback callback);
void spin_subscriber(void* sub);
void destroy_subscriber(void* sub);
```
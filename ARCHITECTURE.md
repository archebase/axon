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

### 1. ROS Recorder (`ros/src/axon_recorder/`)

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

### 2. MCAP Writer (`cpp/axon_mcap/`)

- Thread-safe MCAP file operations
- Zstd/LZ4 compression support
- MCAP validator for file integrity

### 3. Logging Infrastructure (`cpp/axon_logging/`)

- Boost.Log based with async sinks
- Console sink (colors, severity filtering)
- File sink (rotation, JSON/text format)
- ROS sink adapter for RCLCPP/ROSCPP integration
- Environment variable configuration

### 4. Edge Uploader (`cpp/axon_uploader/`)

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

## Performance Characteristics

- **Zero-copy**: Messages moved through queue without copying
- **Direct serialization**: No schema conversion overhead
- **Bounded memory**: Fixed-capacity queues with backpressure
- **Async I/O**: Non-blocking HTTP callbacks and uploads
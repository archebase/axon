# Dagster Integration Design Document

## Executive Summary

This document outlines the design for integrating Dagster, a data orchestration platform, with Axon to enable programmatic control, monitoring, and orchestration of ROS data recording pipelines. The integration provides a unified interface for managing data collection workflows with two distinct flows:

1. **Control Flow**: Human-in-the-loop order creation → Batch generation → Task execution
2. **Data Flow**: Data recording → Edge caching → Inspection (Daft/Ray/Human) → Cloud upload

The system supports hierarchical asset management with scene, sub-scene, and skills metadata, enabling comprehensive data lineage tracking and workflow orchestration.

**Version**: 2.0  
**Date**: 2025
**Status**: Design Phase  
**Author**: Architecture Team

---

## Table of Contents

1. [Overview](#overview)
2. [Current Architecture](#current-architecture)
3. [Integration Goals](#integration-goals)
4. [Dagster Architecture Overview](#dagster-architecture-overview)
5. [Proposed Architecture](#proposed-architecture)
6. [Workflow Design](#workflow-design)
7. [Component Design](#component-design)
8. [Control Flow](#control-flow)
9. [Data Flow](#data-flow)
10. [Asset Management](#asset-management)
11. [Human-in-the-Loop](#human-in-the-loop)
12. [Storage Architecture](#storage-architecture)
13. [Configuration Management](#configuration-management)
14. [Error Handling & Resilience](#error-handling--resilience)
15. [Monitoring & Observability](#monitoring--observability)
16. [Implementation Phases](#implementation-phases)
17. [Testing Strategy](#testing-strategy)
18. [Code Organization](#code-organization)
19. [Deployment Considerations](#deployment-considerations)
20. [Future Enhancements](#future-enhancements)
21. [Appendix](#appendix)

---

## 1. Overview

### 1.1 Background

Axon is a high-performance ROS data recorder that writes ROS messages to Lance format using a C++/Rust FFI bridge. Currently, Axon provides ROS services for basic control (start/stop recording, status checks, config updates), but lacks:

- **Orchestration**: No way to coordinate multiple recording sessions
- **Workflow Management**: No support for complex data collection workflows
- **Monitoring**: Limited observability into pipeline health
- **Scheduling**: No built-in scheduling capabilities
- **Dependency Management**: No way to express dependencies between recording tasks

### 1.2 Why Dagster?

Dagster is a data orchestration platform that provides:

- **Declarative Pipelines**: Define data pipelines as code with clear dependencies
- **Asset Management**: Track data assets and their lineage
- **Observability**: Built-in monitoring, logging, and visualization
- **Scheduling**: Native support for scheduled and event-driven pipelines
- **Type Safety**: Strong typing for configuration and data
- **Extensibility**: Easy to integrate with external systems via ops and resources

### 1.3 Integration Scope

This integration will:

- ✅ Create Dagster ops for Axon recording operations
- ✅ Implement a Dagster resource for Axon ROS service communication
- ✅ Define hierarchical assets (Order → Batch → Task → Dataset) with scene/sub-scene/skills metadata
- ✅ Enable human-in-the-loop workflows for order and batch creation
- ✅ Implement edge caching layer for data staging
- ✅ Support data inspection workflows (Daft, Ray, Human)
- ✅ Enable cloud upload after inspection approval
- ✅ Provide monitoring and observability
- ✅ Support scheduling and event-driven triggers
- ❌ **NOT** modify core Axon C++/Rust code (integration via ROS services)
- ❌ **NOT** replace ROS services (Dagster will use them)

---

## 2. Current Architecture

### 2.1 Axon System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  ROS Interface Layer                         │
│  ┌──────────────┐              ┌──────────────┐            │
│  │  ROS1 Impl   │              │  ROS2 Impl   │            │
│  └──────────────┘              └──────────────┘            │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Message Conversion Layer                        │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ Introspection    │  │ Typed Converters         │        │
│  │ (Dynamic)        │  │ (Optimized)              │        │
│  └──────────────────┘  └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Batch Management Layer                          │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ BatchManager     │  │ SchemaMerger             │        │
│  │ (Arrow Builders) │  │ (Multi-topic)            │        │
│  └──────────────────┘  └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                  FFI Bridge Layer                            │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ C++ Wrapper      │  │ Rust Bridge              │        │
│  │ (LanceWriter)    │  │ (Lance Dataset)          │        │
│  └──────────────────┘  └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Current ROS Services

Axon exposes the following ROS services:

| Service | Request | Response | Purpose |
|---------|---------|----------|---------|
| `/axon/start_recording` | `config_path: string` | `success: bool, message: string` | Start recording with optional config |
| `/axon/stop_recording` | (empty) | `success: bool, message: string, flushed_batches: int32` | Stop recording and flush |
| `/axon/get_status` | (empty) | `is_recording: bool, dataset_path: string, active_topics: string[], batch_sizes: int32[], pending_batches: int32, disk_usage_gb: float64, last_error: string` | Get current status |
| `/axon/update_config` | `config_path: string` | `success: bool, message: string` | Update configuration at runtime |

### 2.3 Current Limitations

1. **Manual Control**: Services must be called manually or via scripts
2. **No Orchestration**: Cannot coordinate multiple recording sessions
3. **No Scheduling**: No built-in scheduling capabilities
4. **Limited Monitoring**: Status must be polled manually
5. **No Workflow Support**: Cannot express complex data collection workflows
6. **No Asset Tracking**: No tracking of dataset lineage or dependencies

---

## 3. Integration Goals

### 3.1 Primary Goals

1. **Control Flow Orchestration**: Enable human-in-the-loop order creation, batch generation, and task execution
2. **Data Flow Management**: Orchestrate data recording → edge caching → inspection → cloud upload
3. **Hierarchical Asset Tracking**: Track Orders, Batches, Tasks, and Datasets with scene/sub-scene/skills metadata
4. **Human-in-the-Loop**: Support admin workflows for creating orders and generating batches
5. **Edge Storage**: Implement edge caching layer for data staging before cloud upload
6. **Inspection Workflows**: Support automated (Daft/Ray) and manual (Human) data inspection
7. **Monitoring**: Provide real-time visibility into both control and data flows
8. **Cloud Integration**: Enable conditional cloud upload after inspection approval

### 3.2 Secondary Goals

1. **Multi-Environment Support**: Support ROS1 and ROS2 environments
2. **Configuration Management**: Centralized configuration via Dagster
3. **Error Recovery**: Automatic retry and error handling
4. **Resource Management**: Efficient resource usage (disk, memory, network)
5. **Extensibility**: Easy to extend with new operations and inspection methods
6. **Data Lineage**: Complete tracking from order creation to cloud storage

### 3.3 Non-Goals

1. **Core Modification**: Will not modify Axon C++/Rust code
2. **ROS Replacement**: Will not replace ROS services
3. **Real-time Processing**: Focus on orchestration, not real-time processing
4. **Data Transformation**: Will not add data transformation capabilities (can be added later)
5. **Cloud Storage Implementation**: Will integrate with existing cloud storage (S3, GCS, etc.) but not implement storage backends

---

## 4. Dagster Architecture Overview

### 4.1 Key Dagster Concepts

- **Ops**: Units of computation (e.g., "start recording", "stop recording")
- **Assets**: Data artifacts (e.g., Lance datasets)
- **Resources**: External dependencies (e.g., ROS service client)
- **Jobs**: Collections of ops that run together
- **Schedules**: Time-based triggers for jobs
- **Sensors**: Event-based triggers for jobs

### 4.2 Dagster Integration Pattern

```
┌─────────────────────────────────────────────────────────────┐
│                    Dagster Layer                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │    Jobs      │  │  Schedules   │  │   Sensors    │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│         ↓                ↓                ↓                  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │                    Ops                                │  │
│  │  start_recording  │  stop_recording  │  get_status  │  │
│  └──────────────────────────────────────────────────────┘  │
│         ↓                                                    │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Resources                                │  │
│  │         AxonROSClient (ROS1/ROS2)                    │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              ROS Service Layer (Axon)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Start/Stop   │  │   Status     │  │   Config     │     │
│  │  Services    │  │   Service    │  │   Service    │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Proposed Architecture

### 5.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Dagster UI                                │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Admin Dashboard  │  Order Management  │  Monitoring │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│              Dagster Daemon                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │  Scheduler   │  │   Run Queue  │  │   Event Log  │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│            Dagster Code Location                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              axon_dagster/                            │  │
│  │  ├── assets.py          # Hierarchical assets        │  │
│  │  ├── ops.py             # All operations              │  │
│  │  ├── resources.py       # Resources (ROS, Storage)   │  │
│  │  ├── jobs.py            # Job definitions             │  │
│  │  ├── schedules.py       # Schedule definitions        │  │
│  │  ├── sensors.py         # Sensor definitions         │  │
│  │  └── config_schema.py   # Configuration schemas      │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│         Control Flow Layer                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Order Ops    │  │ Batch Ops    │  │ Task Ops     │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│         Data Flow Layer                                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Recording    │  │ Edge Cache   │  │ Inspection   │     │
│  │   Ops        │  │   Ops        │  │   Ops        │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│                                                              │
│  ┌──────────────┐  ┌──────────────┐                        │
│  │ Cloud Upload │  │ Storage      │                        │
│  │   Ops        │  │ Resources    │                        │
│  └──────────────┘  └──────────────┘                        │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│         ROS Service Client (Python)                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         axon_dagster/ros_client.py                   │  │
│  │  - ROS1: rospy service client                        │  │
│  │  - ROS2: rclpy service client                       │  │
│  │  - Unified interface for both                       │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│              ROS Network                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │   ROS1       │  │    ROS2      │  │   Axon       │     │
│  │  Master      │  │   DDS        │  │  Recorder    │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│              Storage Layer                                    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │ Edge Cache   │  │ Inspection   │  │ Cloud        │     │
│  │ (Local/Edge) │  │ Storage      │  │ Storage      │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 Component Layers

1. **Dagster Layer**: Orchestration, scheduling, monitoring, UI
2. **Control Flow Layer**: Order → Batch → Task management
3. **Data Flow Layer**: Recording → Edge Cache → Inspection → Cloud
4. **Python ROS Client Layer**: ROS service communication
5. **ROS Network Layer**: ROS1/ROS2 communication
6. **Axon Layer**: Recording and data storage
7. **Storage Layer**: Edge cache, inspection storage, cloud storage

---

## 6. Workflow Design

### 6.1 Two-Flow Architecture

The system operates with two distinct but interconnected flows that serve different purposes:

**Key Principle**: **Control Flow** (lightweight, business logic) is separated from **Data Flow** (heavy, engineering constraints) to ensure robustness and prevent network/storage issues from blocking operations.

#### 6.1.1 Control Flow (The "Brain" - Lightweight, Business Logic)

**Goal**: Instruction & State Management

This flow is **lightweight**. It moves small JSON packets and status updates. It doesn't care about gigabytes of video data; it only cares if the job is done.

```
Admin (Human)
    ↓
Create Order (JSON metadata)
    ↓
Generate Batches (from Order)
    ↓
Generate Tasks (from Batch)
    ↓
Push Config to Robot → Tasks → "ready" status
    ↓
Propagate to Edge Database (SQLite)
    (Task definitions stored locally on edge)
    ↓
Data Collector (Human)
    ↓
Pick up ready task (optional)
    ↓
Use Teleop Device → Start Recording
    ↓
Status Updates: PENDING → EXECUTED → UPLOADED
    (Updated in Edge DB, synced back to Control Flow)
```

**Characteristics**:
- **Lightweight**: Small JSON packets, status updates only
- **Business Logic**: Manages "what needs to happen", not data volume
- **State Management**: Task status transitions (PENDING → EXECUTED → UPLOADED)
- **Edge Database**: Task definitions stored in SQLite on edge device
- **Decoupled**: Not blocked by data transfer or network issues
- **Human-initiated**: Admin creates orders via Dagster UI or API
- **Hierarchical**: Order → Batch → Task structure
- **Metadata-rich**: Each level contains scene, sub-scene, skills information

**Status Flow**:
```
PENDING → READY → EXECUTED → UPLOADED
  ↑        ↑         ↑          ↑
  │        │         │          └─ Data Flow completes upload
  │        │         └────────────── Recording completes
  │        └────────────────────── Config pushed to robot
  └─────────────────────────────── Task created
```

#### 6.1.2 Data Flow (The "Muscle" - Heavy, Engineering Constraints)

**Goal**: Volume, Integrity & Movement

This flow is **heavy**. It deals with the actual Lance files (gigabytes). It is **decoupled** from the Control Flow to prevent network lag from blocking the robot's operation.

```
Task Execution (Triggered by Control Flow)
    ↓
Start Recording (Axon via ROS)
    ↓
Data Recording (ROS Messages → Lance Dataset)
    (Hot Storage: Local on robot/edge)
    ↓
Edge Cache Storage (Data Lake Layer)
    (Accessible for immediate replay or QC on edge)
    ↓
Inspection (Daft/Ray/Human)
    (Optional: Quality check before upload)
    ↓
Bandwidth Sensor (Dagster)
    (Checks network conditions)
    ↓
Cloud Upload (if network available)
    (Pipes data to Cloud Data Lake)
    ↓
Upload Complete → Update Control Flow Status
```

**Characteristics**:
- **Heavy**: Deals with gigabytes of Lance files
- **Engineering Constraints**: Manages bandwidth, storage, IO
- **Decoupled**: Not blocked by Control Flow issues
- **Staged**: Data flows through edge cache before cloud
- **Bandwidth-Aware**: Dagster sensor checks network before upload
- **Resilient**: Can continue even if Control Flow is paused
- **Traceable**: Complete lineage from recording to cloud

**Storage Layers**:
1. **Hot Storage**: Local on robot during recording
2. **Edge Cache**: Data Lake layer on edge device
3. **Cloud Data Lake**: Final destination after upload

### 6.2 Flow Integration: Dagster as the Bridge

**Dagster's Unique Role**: Dagster sits right in the middle of these two flows, acting as the bridge between Control Flow and Data Flow.

```
┌─────────────────────────────────────────────────────────────┐
│                    Control Flow                              │
│              (Lightweight, Business Logic)                  │
│                                                              │
│  Order → Batch → Task → Edge DB (SQLite)                    │
│                              ↓                               │
│                    Status: PENDING → EXECUTED → UPLOADED    │
└─────────────────────────────────────────────────────────────┘
                         ↕
              ┌──────────────────────┐
              │   Dagster (Bridge)    │
              │                       │
              │  • Sensors watch      │
              │    Task DB            │
              │  • Triggers Data     │
              │    Flow ops           │
              │  • Updates status    │
              │    back to Control    │
              └──────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│                    Data Flow                                  │
│           (Heavy, Engineering Constraints)                  │
│                                                              │
│  Recording → Edge Cache → Inspection → Cloud Upload         │
│     (Lance files, gigabytes)                                 │
└─────────────────────────────────────────────────────────────┘
```

**How Dagster Bridges the Flows**:

1. **Listens to Control Flow**: 
   - Sensor watches Edge Database (SQLite) for task status changes
   - Detects when tasks move to "ready" or "executed" status

2. **Triggers Data Flow**: 
   - Starts Recording Op when task is ready
   - Initiates upload when task is executed
   - Manages bandwidth-aware upload scheduling

3. **Reports back to Control Flow**: 
   - Updates Edge DB status when Data Flow completes
   - Changes status: EXECUTED → UPLOADED
   - Provides visibility into data flow progress

**Task State Transitions** (with Control Flow integration):
```
Control Flow:          Data Flow:
pending → ready → executing → completed/failed
  ↑        ↑         ↑
  │        │         └─ Teleop triggers recording
  │        │        └──────────── Data collector picks up (optional)
  │         └───────────────────── Config pushed to robot, ready for teleop
  └─────────────────────────────── Task created from batch

Status in Edge DB:
PENDING → READY → EXECUTED → UPLOADED
  ↑        ↑         ↑          ↑
  │        │         │          └─ Dagster updates after upload
  │        │         └──────────── Recording completes
  │        └─────────────────────── Config pushed
  └─────────────────────────────── Task created
```

**Benefits of Separation**:

1. **If Cloud goes down**: 
   - ✅ Control Flow pauses (no new orders)
   - ✅ Data Flow continues (recording and caching work independently)

2. **If Network is slow**: 
   - ✅ Data Flow pauses (upload waits for bandwidth)
   - ✅ Control Flow unaffected (can create new orders, update status)

3. **If Edge Database is slow**: 
   - ✅ Data Flow continues (recording doesn't depend on DB)
   - ⚠️ Control Flow may be slower (status updates delayed)

4. **Resilience**: 
   - Each flow can operate independently
   - Failures in one flow don't cascade to the other

---

## 7. Component Design

### 7.1 ROS Client Resource

**File**: `axon_dagster/resources.py`

```python
@resource(
    config_schema={
        "ros_version": Field(str, default_value="ros2", description="ROS version: 'ros1' or 'ros2'"),
        "namespace": Field(str, default_value="axon", description="ROS namespace for Axon services"),
        "timeout_seconds": Field(float, default_value=30.0, description="Service call timeout"),
        "retry_attempts": Field(int, default_value=3, description="Number of retry attempts"),
    }
)
def axon_ros_client(context) -> AxonROSClient:
    """Resource providing ROS service client for Axon."""
    # Implementation details below
```

**Responsibilities**:
- Initialize ROS connection (ROS1 or ROS2)
- Provide service client interface
- Handle service call timeouts and retries
- Manage ROS node lifecycle

**Interface**:
```python
class AxonROSClient:
    def start_recording(self, config_path: Optional[str] = None) -> StartRecordingResponse
    def stop_recording(self) -> StopRecordingResponse
    def get_status(self) -> GetStatusResponse
    def update_config(self, config_path: str) -> UpdateConfigResponse
```

### 7.2 Edge Storage Resource

**File**: `axon_dagster/resources.py`

```python
@resource(
    config_schema={
        "base_path": Field(str, default_value="/data/edge_cache", description="Base path for edge cache storage"),
        "storage_type": Field(str, default_value="local", description="Storage type: 'local', 'nfs', 's3'"),
        "max_size_gb": Field(float, default_value=1000.0, description="Maximum edge cache size in GB"),
        "cleanup_policy": Field(str, default_value="lru", description="Cleanup policy: 'lru', 'fifo', 'manual'"),
    }
)
def edge_storage_resource(context) -> EdgeStorage:
    """Resource providing edge cache storage interface."""
    return EdgeStorage(
        base_path=context.resource_config["base_path"],
        storage_type=context.resource_config["storage_type"],
        max_size_gb=context.resource_config["max_size_gb"],
        cleanup_policy=context.resource_config["cleanup_policy"],
    )
```

**Interface**:
```python
class EdgeStorage:
    def copy_to_edge_cache(self, source_path: str, dest_path: str) -> None
    def get_size(self, path: str) -> int  # Returns size in bytes
    def exists(self, path: str) -> bool
    def delete(self, path: str) -> None
    def list_datasets(self, pattern: Optional[str] = None) -> List[str]
    def cleanup(self) -> None  # Cleanup based on policy
```

### 7.3 Cloud Storage Resource

**File**: `axon_dagster/resources.py`

```python
@resource(
    config_schema={
        "storage_type": Field(str, is_required=True, description="Cloud storage type: 's3', 'gcs', 'azure'"),
        "bucket": Field(str, is_required=True, description="Cloud storage bucket/container name"),
        "base_path": Field(str, default_value="", description="Base path prefix in cloud storage"),
        "credentials": Field(dict, is_required=False, description="Cloud storage credentials"),
    }
)
def cloud_storage_resource(context) -> CloudStorage:
    """Resource providing cloud storage interface."""
    return CloudStorage(
        storage_type=context.resource_config["storage_type"],
        bucket=context.resource_config["bucket"],
        base_path=context.resource_config["base_path"],
        credentials=context.resource_config.get("credentials", {}),
    )
```

**Interface**:
```python
class CloudStorage:
    def upload(self, local_path: str, cloud_path: str) -> None
    def download(self, cloud_path: str, local_path: str) -> None
    def get_size(self, cloud_path: str) -> int  # Returns size in bytes
    def exists(self, cloud_path: str) -> bool
    def delete(self, cloud_path: str) -> None
    def list_datasets(self, prefix: Optional[str] = None) -> List[str]
```

### 6.2 Recording Operations

**File**: `axon_dagster/ops.py`

#### 6.2.1 Start Recording Op

```python
@op(
    config_schema={
        "config_path": Field(str, is_required=False, description="Path to YAML config file"),
        "dataset_path": Field(str, is_required=False, description="Override dataset path"),
        "topics": Field(list, is_required=False, description="List of topics to record"),
    },
    required_resource_keys={"axon_ros_client"},
)
def start_recording_op(context) -> Dict[str, Any]:
    """Start Axon recording session."""
    # Implementation
```

#### 6.2.2 Stop Recording Op

```python
@op(
    config_schema={
        "wait_for_flush": Field(bool, default_value=True, description="Wait for batch flush"),
        "timeout_seconds": Field(float, default_value=60.0, description="Flush timeout"),
    },
    required_resource_keys={"axon_ros_client"},
)
def stop_recording_op(context) -> Dict[str, Any]:
    """Stop Axon recording session."""
    # Implementation
```

#### 6.2.3 Get Status Op

```python
@op(
    required_resource_keys={"axon_ros_client"},
)
def get_status_op(context) -> Dict[str, Any]:
    """Get current Axon recording status."""
    # Implementation
```

#### 6.2.4 Update Config Op

```python
@op(
    config_schema={
        "config_path": Field(str, is_required=True, description="Path to new config file"),
    },
    required_resource_keys={"axon_ros_client"},
)
def update_config_op(context) -> Dict[str, Any]:
    """Update Axon configuration."""
    # Implementation
```

#### 6.2.5 Wait for Recording Op

```python
@op(
    config_schema={
        "duration_seconds": Field(float, is_required=True, description="Recording duration"),
        "check_interval_seconds": Field(float, default_value=5.0, description="Status check interval"),
    },
    required_resource_keys={"axon_ros_client"},
)
def wait_for_recording_op(context) -> Dict[str, Any]:
    """Wait for specified recording duration, monitoring status."""
    # Implementation
```

### 6.3 Dataset Assets

**File**: `axon_dagster/assets.py`

```python
@asset(
    config_schema={
        "dataset_path": Field(str, is_required=True, description="Path to Lance dataset"),
    },
    required_resource_keys={"axon_ros_client"},
)
def lance_dataset_asset(context) -> str:
    """Asset representing a Lance dataset created by Axon."""
    # Verify dataset exists and return path
    # Track metadata (size, topics, time range, etc.)
```

**Asset Metadata**:
- Dataset path
- Creation timestamp
- Size (bytes)
- Topics recorded
- Time range (start/end)
- Schema information
- Recording configuration

### 6.4 Jobs

**File**: `axon_dagster/jobs.py`

#### 6.4.1 Simple Recording Job

```python
@job(
    config={
        "ops": {
            "start_recording_op": {
                "config": {
                    "config_path": "/path/to/config.yaml",
                }
            },
            "wait_for_recording_op": {
                "config": {
                    "duration_seconds": 3600.0,  # 1 hour
                }
            },
            "stop_recording_op": {
                "config": {}
            },
        }
    },
    resource_defs={"axon_ros_client": axon_ros_client},
)
def simple_recording_job():
    """Simple job: start, wait, stop."""
    status = get_status_op()
    start = start_recording_op()
    wait = wait_for_recording_op(start)
    stop = stop_recording_op(wait)
    final_status = get_status_op(stop)
    return final_status
```

#### 6.4.2 Multi-Session Recording Job

```python
@job(
    resource_defs={"axon_ros_client": axon_ros_client},
)
def multi_session_recording_job():
    """Record multiple sessions with different configurations."""
    # Session 1: Camera only
    session1 = start_recording_op.alias("session1_camera")(
        config={"config_path": "/configs/camera_only.yaml"}
    )
    wait1 = wait_for_recording_op.alias("wait1")(session1, config={"duration_seconds": 1800.0})
    stop1 = stop_recording_op.alias("stop1")(wait1)
    
    # Session 2: Lidar only
    session2 = start_recording_op.alias("session2_lidar")(
        stop1,
        config={"config_path": "/configs/lidar_only.yaml"}
    )
    wait2 = wait_for_recording_op.alias("wait2")(session2, config={"duration_seconds": 1800.0})
    stop2 = stop_recording_op.alias("stop2")(wait2)
    
    return stop2
```

#### 6.4.3 Conditional Recording Job

```python
@job(
    resource_defs={"axon_ros_client": axon_ros_client},
)
def conditional_recording_job():
    """Start recording only if certain conditions are met."""
    status = get_status_op()
    
    # Check disk space, system health, etc.
    # Start recording conditionally
    start = start_recording_op(status)
    # ...
```

### 6.5 Schedules

**File**: `axon_dagster/schedules.py`

#### 6.5.1 Daily Recording Schedule

```python
@schedule(
    job=simple_recording_job,
    cron_schedule="0 9 * * *",  # 9 AM daily
    default_status=DefaultScheduleStatus.RUNNING,
)
def daily_recording_schedule(context):
    """Schedule daily recording sessions."""
    return RunRequest(
        run_key=f"daily_recording_{context.scheduled_execution_time.strftime('%Y%m%d')}",
        run_config={
            "ops": {
                "start_recording_op": {
                    "config": {
                        "config_path": "/configs/daily_recording.yaml",
                    }
                },
                "wait_for_recording_op": {
                    "config": {
                        "duration_seconds": 28800.0,  # 8 hours
                    }
                },
            }
        },
        tags={"recording_type": "daily", "duration": "8h"},
    )
```

#### 6.5.2 Hourly Short Recording Schedule

```python
@schedule(
    job=simple_recording_job,
    cron_schedule="0 * * * *",  # Every hour
    default_status=DefaultScheduleStatus.RUNNING,
)
def hourly_recording_schedule(context):
    """Schedule hourly short recording sessions."""
    return RunRequest(
        run_key=f"hourly_recording_{context.scheduled_execution_time.strftime('%Y%m%d_%H%M')}",
        run_config={
            "ops": {
                "start_recording_op": {
                    "config": {
                        "config_path": "/configs/hourly_recording.yaml",
                    }
                },
                "wait_for_recording_op": {
                    "config": {
                        "duration_seconds": 300.0,  # 5 minutes
                    }
                },
            }
        },
        tags={"recording_type": "hourly", "duration": "5m"},
    )
```

### 6.6 Sensors

**File**: `axon_dagster/sensors.py`

#### 6.6.1 Disk Space Sensor

```python
@sensor(
    job=simple_recording_job,
    minimum_interval_seconds=60,
)
def disk_space_sensor(context):
    """Trigger recording when disk space is available."""
    status = context.resources.axon_ros_client.get_status()
    
    # Check disk space
    if status.disk_usage_gb < 50.0:  # Less than 50GB used
        return RunRequest(
            run_key=f"disk_space_recording_{int(time.time())}",
            run_config={
                "ops": {
                    "start_recording_op": {
                        "config": {
                            "config_path": "/configs/default.yaml",
                        }
                    },
                    "wait_for_recording_op": {
                        "config": {
                            "duration_seconds": 3600.0,
                        }
                    },
                }
            },
            tags={"trigger": "disk_space"},
        )
    return SkipReason("Insufficient disk space")
```

#### 6.6.2 Edge Database Task Status Sensor

```python
@sensor(
    job=data_flow_job,
    minimum_interval_seconds=5,
)
def edge_db_task_status_sensor(context):
    """
    Watch Edge Database (SQLite) for task status changes.
    This is how Dagster listens to Control Flow.
    
    When task status changes:
    - READY → Trigger recording preparation
    - EXECUTED → Trigger upload (if bandwidth available)
    """
    import sqlite3
    
    # Connect to Edge Database
    edge_db_path = context.resource_config.get("edge_db_path", "/data/edge/tasks.db")
    conn = sqlite3.connect(edge_db_path)
    cursor = conn.cursor()
    
    # Query tasks that need attention
    # Tasks in EXECUTED status that haven't been uploaded
    cursor.execute("""
        SELECT task_id, status, dataset_path, created_at
        FROM tasks
        WHERE status = 'EXECUTED' 
        AND uploaded = 0
        ORDER BY created_at ASC
        LIMIT 10
    """)
    
    tasks = cursor.fetchall()
    conn.close()
    
    if tasks:
        # Trigger upload job for each task
        run_requests = []
        for task_id, status, dataset_path, created_at in tasks:
            run_requests.append(
                RunRequest(
                    run_key=f"upload_task_{task_id}_{int(time.time())}",
                    run_config={
                        "ops": {
                            "cloud_upload_op": {
                                "config": {
                                    "task_id": task_id,
                                    "dataset_path": dataset_path,
                                }
                            }
                        }
                    },
                    tags={"task_id": task_id, "trigger": "edge_db"},
                )
            )
        return run_requests
    
    return SkipReason("No tasks ready for upload")
```

#### 6.6.3 Bandwidth Sensor

```python
@sensor(
    job=cloud_upload_job,
    minimum_interval_seconds=60,
)
def bandwidth_sensor(context):
    """
    Check network bandwidth before triggering upload.
    This prevents Data Flow from overwhelming slow networks.
    
    Only triggers upload if:
    - Network bandwidth is sufficient
    - Network is stable
    - No other uploads in progress
    """
    import subprocess
    import json
    
    # Check current network bandwidth
    # This could use tools like iperf, speedtest, or custom network monitoring
    try:
        # Example: Check upload speed
        result = subprocess.run(
            ["check_bandwidth"],  # Custom script or tool
            capture_output=True,
            text=True,
            timeout=10
        )
        bandwidth_info = json.loads(result.stdout)
        
        min_bandwidth_mbps = context.resource_config.get("min_bandwidth_mbps", 10.0)
        
        if bandwidth_info["upload_mbps"] >= min_bandwidth_mbps:
            # Bandwidth is sufficient, check for tasks to upload
            return edge_db_task_status_sensor(context)
        else:
            return SkipReason(
                f"Insufficient bandwidth: {bandwidth_info['upload_mbps']:.2f} Mbps "
                f"(required: {min_bandwidth_mbps} Mbps)"
            )
    except Exception as e:
        context.log.warning(f"Bandwidth check failed: {e}")
        return SkipReason("Bandwidth check unavailable")
```

#### 6.6.4 ROS Topic Availability Sensor

```python
@sensor(
    job=simple_recording_job,
    minimum_interval_seconds=30,
)
def topic_availability_sensor(context):
    """Trigger recording when required topics become available."""
    # Check if required topics are publishing
    # If yes, start recording
    # Implementation would use ROS topic list/echo
```

---

## 8. Control Flow

### 8.1 Control Flow Overview

The control flow represents the human-in-the-loop orchestration of data collection orders. It follows a hierarchical structure:

```
Order (Human-created)
  ├── Metadata: scene, sub-scene, skills, requirements
  ├── Status: pending, active, completed, failed
  └── Batches (Generated from Order)
      ├── Batch 1
      │   ├── Metadata: scene, sub-scene, skills (inherited from Order)
      │   ├── Status: pending, ready, executing, completed
      │   └── Tasks (Generated from Batch)
      │       ├── Task 1 → Triggers Data Flow
      │       ├── Task 2 → Triggers Data Flow
      │       └── Task N → Triggers Data Flow
      └── Batch N
```

### 8.2 Order Creation (Human-in-the-Loop)

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "order_id": Field(str, is_required=True, description="Unique order identifier"),
        "scene": Field(str, is_required=True, description="Scene name (e.g., 'warehouse', 'outdoor')"),
        "sub_scene": Field(str, is_required=False, description="Sub-scene name (e.g., 'aisle_1', 'parking_lot')"),
        "skills": Field(list, is_required=True, description="List of skills to record (e.g., ['navigation', 'manipulation'])"),
        "requirements": Field(dict, is_required=False, description="Additional requirements (topics, duration, etc.)"),
        "priority": Field(int, default_value=0, description="Order priority"),
    },
)
def create_order_op(context) -> Dict[str, Any]:
    """
    Create a new data collection order.
    This is typically triggered by an admin via Dagster UI or API.
    """
    order_id = context.op_config["order_id"]
    scene = context.op_config["scene"]
    sub_scene = context.op_config.get("sub_scene", "")
    skills = context.op_config["skills"]
    requirements = context.op_config.get("requirements", {})
    priority = context.op_config.get("priority", 0)
    
    # Create order asset
    order_metadata = {
        "order_id": order_id,
        "scene": scene,
        "sub_scene": sub_scene,
        "skills": skills,
        "requirements": requirements,
        "priority": priority,
        "status": "pending",
        "created_at": datetime.now().isoformat(),
        "created_by": context.run_id,  # Or user ID if available
    }
    
    # Store order (database, file, or Dagster asset)
    # This would integrate with a persistence layer
    
    context.log.info(f"Created order {order_id} for scene={scene}, skills={skills}")
    
    return {
        "order_id": order_id,
        "status": "pending",
        "metadata": order_metadata,
    }
```

### 8.3 Batch Generation

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "order_id": Field(str, is_required=True, description="Order ID to generate batches for"),
        "batch_strategy": Field(str, default_value="auto", description="'auto' or 'manual' batch generation"),
        "batch_config": Field(dict, is_required=False, description="Batch generation configuration"),
    },
)
def generate_batches_op(context, order_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generate batches from an order.
    Batches can be generated automatically based on order requirements
    or manually specified by the admin.
    """
    order_id = order_data["order_id"]
    batch_strategy = context.op_config.get("batch_strategy", "auto")
    batch_config = context.op_config.get("batch_config", {})
    
    # Extract order metadata
    scene = order_data["metadata"]["scene"]
    sub_scene = order_data["metadata"].get("sub_scene", "")
    skills = order_data["metadata"]["skills"]
    requirements = order_data["metadata"].get("requirements", {})
    
    batches = []
    
    if batch_strategy == "auto":
        # Automatic batch generation based on skills and requirements
        # Example: One batch per skill, or one batch per time window
        for skill in skills:
            batch_id = f"{order_id}_batch_{skill}_{len(batches) + 1}"
            batch = {
                "batch_id": batch_id,
                "order_id": order_id,
                "scene": scene,
                "sub_scene": sub_scene,
                "skills": [skill],  # One skill per batch
                "status": "pending",
                "created_at": datetime.now().isoformat(),
            }
            batches.append(batch)
    else:
        # Manual batch generation from batch_config
        # batch_config would contain explicit batch definitions
        for batch_def in batch_config.get("batches", []):
            batch_id = f"{order_id}_batch_{len(batches) + 1}"
            batch = {
                "batch_id": batch_id,
                "order_id": order_id,
                "scene": scene,
                "sub_scene": sub_scene,
                "skills": batch_def.get("skills", skills),
                "status": "pending",
                "created_at": datetime.now().isoformat(),
            }
            batches.append(batch)
    
    context.log.info(f"Generated {len(batches)} batches for order {order_id}")
    
    return {
        "order_id": order_id,
        "batches": batches,
        "batch_count": len(batches),
    }
```

### 8.4 Task Generation

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "batch_id": Field(str, is_required=True, description="Batch ID to generate tasks for"),
        "task_config": Field(dict, is_required=False, description="Task generation configuration"),
    },
)
def generate_tasks_op(context, batch_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Generate tasks from a batch.
    Tasks are created in "ready" status, waiting for data collector to pick them up.
    """
    batch_id = batch_data["batch_id"]
    order_id = batch_data["order_id"]
    scene = batch_data["scene"]
    sub_scene = batch_data.get("sub_scene", "")
    skills = batch_data["skills"]
    task_config = context.op_config.get("task_config", {})
    
    tasks = []
    
    # Generate tasks based on batch configuration
    # Example: One task per recording session, or multiple tasks for different scenarios
    task_count = task_config.get("task_count", 1)
    task_duration = task_config.get("duration_seconds", 3600.0)
    task_topics = task_config.get("topics", [])
    
    for i in range(task_count):
        task_id = f"{batch_id}_task_{i + 1}"
        task = {
            "task_id": task_id,
            "batch_id": batch_id,
            "order_id": order_id,
            "scene": scene,
            "sub_scene": sub_scene,
            "skills": skills,
            "status": "ready",  # Tasks start in "ready" status for data collector pickup
            "duration_seconds": task_duration,
            "topics": task_topics,
            "created_at": datetime.now().isoformat(),
            "ready_at": datetime.now().isoformat(),  # When task became ready
            "assigned_to": None,  # Will be set when data collector picks up task
        }
        tasks.append(task)
    
    context.log.info(f"Generated {len(tasks)} tasks for batch {batch_id} (status: ready)")
    
    return {
        "batch_id": batch_id,
        "tasks": tasks,
        "task_count": len(tasks),
    }
```

### 8.5 Task Ready Status and Data Collector Workflow

**File**: `axon_dagster/ops.py`

#### 8.5.1 Push Config to Device and Mark Task as Ready

```python
@op(
    config_schema={
        "task_id": Field(str, is_required=True, description="Task ID to push to device"),
        "robot_id": Field(str, is_required=False, description="Target robot/device ID"),
    },
    required_resource_keys={"axon_ros_client"},
)
def push_config_to_device_op(context, task_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Push task configuration to robot/device side and mark task as ready.
    
    "Ready" status means:
    - Configuration has been successfully pushed to the robot
    - Robot is ready to receive teleop command to start recording
    - Data collector can now use teleop device to trigger recording
    """
    task_id = task_data["task_id"]
    robot_id = context.op_config.get("robot_id", "default")
    
    # Generate configuration for this task
    task_config = {
        "dataset": {
            "path": generate_dataset_path(task_data),
            "mode": "create",
        },
        "topics": task_data.get("topics", []),
        "metadata": {
            "task_id": task_id,
            "scene": task_data["scene"],
            "sub_scene": task_data.get("sub_scene", ""),
            "skills": task_data["skills"],
            "duration_seconds": task_data.get("duration_seconds", 3600.0),
        },
    }
    
    # Save config file
    config_path = f"/tmp/axon_config_{task_id}.yaml"
    with open(config_path, "w") as f:
        yaml.dump(task_config, f)
    
    # Push config to robot via ROS service (UpdateConfig)
    client = context.resources.axon_ros_client
    
    # Option 1: Update config on robot (if robot is already running Axon)
    response = client.update_config(config_path=config_path)
    
    if not response.success:
        raise Failure(f"Failed to push config to robot {robot_id}: {response.message}")
    
    # Option 2: Or store config on robot filesystem for later use
    # This could be done via:
    # - ROS service to write file on robot
    # - SSH/SCP to copy config file
    # - ROS parameter server
    
    context.log.info(
        f"Configuration pushed to robot {robot_id} for task {task_id}. "
        f"Config path: {config_path}"
    )
    
    # Update task status to "ready" (config is on device)
    updated_task = {
        **task_data,
        "status": "ready",
        "ready_at": datetime.now().isoformat(),
        "robot_id": robot_id,
        "config_path": config_path,
        "config_pushed": True,
    }
    
    # Store updated task (database, file, or Dagster asset)
    # Make it visible to data collectors via UI/API
    
    return updated_task
```

#### 8.5.2 Data Collector Picks Up Task

```python
@op(
    config_schema={
        "task_id": Field(str, is_required=True, description="Task ID to pick up"),
        "data_collector_id": Field(str, is_required=True, description="Data collector identifier"),
    },
)
def pickup_task_op(context, task_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Data collector picks up a ready task.
    This transitions the task from "ready" to "assigned".
    """
    task_id = task_data["task_id"]
    data_collector_id = context.op_config["data_collector_id"]
    
    if task_data["status"] != "ready":
        raise Failure(f"Task {task_id} is not in ready status (current: {task_data['status']})")
    
    # Assign task to data collector
    updated_task = {
        **task_data,
        "status": "assigned",
        "assigned_to": data_collector_id,
        "assigned_at": datetime.now().isoformat(),
    }
    
    context.log.info(f"Task {task_id} assigned to data collector {data_collector_id}")
    
    return updated_task
```

#### 8.5.3 Teleop Device Integration

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "task_id": Field(str, is_required=True, description="Task ID"),
        "teleop_device_id": Field(str, is_required=True, description="Teleop device identifier"),
        "teleop_command": Field(str, default_value="start_recording", description="Teleop command"),
    },
    required_resource_keys={"axon_ros_client"},
)
def teleop_start_recording_op(context, task_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Data collector uses teleop device to start recording.
    This triggers the automated data flow.
    
    The teleop device sends a command (e.g., button press) which:
    1. Triggers this op
    2. Calls Axon ROS service to start recording (using config already on robot)
    3. Begins the automated data flow
    
    Note: Configuration should already be on the robot (task status = "ready").
    This op just sends the start command.
    """
    task_id = task_data["task_id"]
    teleop_device_id = context.op_config["teleop_device_id"]
    teleop_command = context.op_config.get("teleop_command", "start_recording")
    
    if task_data["status"] not in ["assigned", "ready"]:
        raise Failure(
            f"Task {task_id} cannot be executed (status: {task_data['status']}). "
            f"Task must be in 'ready' status (config pushed to robot)."
        )
    
    if not task_data.get("config_pushed", False):
        raise Failure(
            f"Task {task_id} configuration not pushed to robot. "
            f"Cannot start recording."
        )
    
    context.log.info(
        f"Teleop device {teleop_device_id} triggered {teleop_command} for task {task_id}"
    )
    
    # Update task status to executing
    updated_task = {
        **task_data,
        "status": "executing",
        "execution_started_at": datetime.now().isoformat(),
        "teleop_device_id": teleop_device_id,
    }
    
    # Start recording via Axon ROS service
    # Config is already on robot, so we can use empty config_path or the one already pushed
    client = context.resources.axon_ros_client
    
    # Option 1: Use config already on robot (empty config_path uses robot's current config)
    # Option 2: Use the config_path that was pushed earlier
    config_path = task_data.get("config_path", "")
    
    # Start recording (robot uses config that was already pushed)
    response = client.start_recording(config_path=config_path)
    
    if not response.success:
        raise Failure(f"Failed to start recording: {response.message}")
    
    context.log.info(
        f"Recording started for task {task_id} via teleop device. "
        f"Using config already on robot: {config_path}"
    )
    
    return {
        **updated_task,
        "recording_started": True,
        "dataset_path": response.dataset_path,
    }
```

#### 8.5.4 Complete Task Execution Flow

```python
@op(
    required_resource_keys={"axon_ros_client", "edge_storage"},
)
def execute_task_op(context, task_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Complete task execution after teleop triggers recording.
    This op coordinates the entire data flow for a single task.
    
    Note: This is called AFTER teleop_start_recording_op has started recording.
    """
    task_id = task_data["task_id"]
    scene = task_data["scene"]
    sub_scene = task_data.get("sub_scene", "")
    skills = task_data["skills"]
    duration = task_data.get("duration_seconds", 3600.0)
    
    context.log.info(f"Executing task {task_id} for scene={scene}, skills={skills}")
    
    # Task execution continues with automated data flow:
    # 1. Wait for recording (monitor recording)
    # 2. Stop recording (when duration reached or teleop stop command)
    # 3. Move to edge cache
    # 4. Trigger inspection
    
    return {
        "task_id": task_id,
        "status": "executing",
        "data_flow_triggered": True,
    }
```

### 8.6 Complete Control Flow Job

**File**: `axon_dagster/jobs.py`

```python
@job(
    resource_defs={
        "axon_ros_client": axon_ros_client,
        "edge_storage": edge_storage_resource,
    },
)
def order_processing_job():
    """
    Complete control flow: Order → Batches → Tasks → Ready → Data Collector → Teleop → Data Flow
    
    This job handles the automated parts:
    1. Create order
    2. Generate batches
    3. Generate tasks
    4. Mark tasks as ready
    
    Then waits for human data collector to:
    5. Pick up task
    6. Use teleop device to start recording
    7. Automated data flow continues
    """
    # Step 1: Create Order (Admin-triggered)
    order = create_order_op()
    
    # Step 2: Generate Batches from Order
    batches = generate_batches_op(order)
    
    # Step 3: Generate Tasks from Batches
    tasks = generate_tasks_op(batches)
    
    # Step 4: Push Config to Device and Mark Tasks as Ready (automated)
    ready_tasks = push_config_to_device_op(tasks)
    
    # At this point:
    # - Configuration has been pushed to robot/device
    # - Tasks are in "ready" status
    # - Robot is ready to receive teleop command to start recording
    # - Waiting for data collector to use teleop device
    
    # Step 5: Data Collector picks up task (human-triggered via UI)
    # This would be triggered separately when data collector selects a task
    # assigned_task = pickup_task_op(ready_tasks)
    
    # Step 6: Teleop device triggers recording (human-triggered via teleop)
    # This would be triggered when data collector uses teleop device
    # recording_started = teleop_start_recording_op(assigned_task)
    
    # Step 7: Automated data flow continues
    # This is handled by the data_flow_job which is triggered after recording starts
    
    return ready_tasks
```

**Separate Job for Data Collector Actions**:

```python
@job(
    resource_defs={
        "axon_ros_client": axon_ros_client,
    },
)
def data_collector_workflow_job():
    """
    Job for data collector actions (triggered by data collector via UI/teleop).
    """
    # This job is triggered when:
    # 1. Data collector picks up a task (via UI)
    # 2. Data collector uses teleop device (via teleop command)
    
    # Pick up task
    assigned_task = pickup_task_op()
    
    # Teleop triggers recording
    recording_started = teleop_start_recording_op(assigned_task)
    
    return recording_started
```

### 8.7 Edge Database (SQLite)

**Purpose**: Lightweight local database on edge device/robot for task state management in Control Flow.

**Location**: `/data/edge/tasks.db` (on robot/edge device)

**Schema**:

```sql
-- Tasks table
CREATE TABLE tasks (
    task_id TEXT PRIMARY KEY,
    order_id TEXT NOT NULL,
    batch_id TEXT NOT NULL,
    scene TEXT NOT NULL,
    sub_scene TEXT,
    skills TEXT,  -- JSON array
    status TEXT NOT NULL,  -- PENDING, READY, EXECUTED, UPLOADED, FAILED
    config_path TEXT,
    dataset_path TEXT,
    robot_id TEXT,
    data_collector_id TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    ready_at TIMESTAMP,
    executed_at TIMESTAMP,
    uploaded_at TIMESTAMP,
    uploaded INTEGER DEFAULT 0,  -- 0 = not uploaded, 1 = uploaded
    metadata TEXT  -- JSON object
);

-- Orders table
CREATE TABLE orders (
    order_id TEXT PRIMARY KEY,
    scene TEXT NOT NULL,
    sub_scene TEXT,
    skills TEXT,  -- JSON array
    status TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    created_by TEXT
);

-- Batches table
CREATE TABLE batches (
    batch_id TEXT PRIMARY KEY,
    order_id TEXT NOT NULL,
    scene TEXT NOT NULL,
    sub_scene TEXT,
    skills TEXT,  -- JSON array
    status TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_tasks_status ON tasks(status);
CREATE INDEX idx_tasks_uploaded ON tasks(uploaded);
CREATE INDEX idx_tasks_robot_id ON tasks(robot_id);
```

**Usage in Control Flow**:

1. **Task Creation**: When tasks are generated, they're inserted into Edge DB
2. **Status Updates**: Task status changes are written to Edge DB
3. **Dagster Sensors**: Watch Edge DB for status changes
4. **Data Flow Completion**: Updates Edge DB when upload completes

**Example Operations**:

```python
# Insert task
INSERT INTO tasks (task_id, order_id, batch_id, scene, skills, status)
VALUES (?, ?, ?, ?, ?, 'PENDING');

# Update status to READY (after config push)
UPDATE tasks 
SET status = 'READY', ready_at = CURRENT_TIMESTAMP, config_path = ?
WHERE task_id = ?;

# Update status to EXECUTED (after recording)
UPDATE tasks 
SET status = 'EXECUTED', executed_at = CURRENT_TIMESTAMP, dataset_path = ?
WHERE task_id = ?;

# Update status to UPLOADED (after cloud upload)
UPDATE tasks 
SET status = 'UPLOADED', uploaded_at = CURRENT_TIMESTAMP, uploaded = 1
WHERE task_id = ?;
```

**Benefits**:
- **Lightweight**: SQLite is perfect for edge devices
- **Local**: No network dependency for state management
- **Fast**: Quick reads/writes for status updates
- **Persistent**: Survives device reboots
- **Queryable**: Easy to query task status, find ready tasks, etc.

### 8.8 Control Flow State Management

Orders, batches, and tasks need state management:

```python
# State transitions
Order: pending → active → completed/failed
Batch: pending → ready → executing → completed/failed
Task: pending → ready → assigned → executing → completed/failed
      ↑         ↑        ↑          ↑
      │         │        │          └─ Teleop triggers recording
      │         │        └──────────── Data collector picks up (optional)
      │         └───────────────────── Config pushed to robot, ready for teleop
      └─────────────────────────────── Task created from batch

# Detailed Task State Transitions:
# 1. pending: Task created, configuration not yet pushed to device
# 2. ready: Configuration pushed to robot/device, ready for teleop command
#           - Config file is on robot
#           - Robot is ready to receive start command
#           - Data collector can use teleop device
# 3. assigned: Data collector has picked up the task (optional state)
# 4. executing: Teleop device triggered recording, data flow active
# 5. completed: Task finished successfully
# 6. failed: Task failed (error during execution)

# State persistence
# Options:
# 1. Dagster asset metadata
# 2. External database (PostgreSQL, etc.)
# 3. File-based storage
# 4. Dagster run storage
```

### 8.7 Data Collector Workflow

**Data Collector Interface** (Dagster UI or separate app):

1. **View Ready Tasks**: Data collector sees list of tasks in "ready" status
   - "Ready" means: Configuration is already on the robot
   - Robot is waiting for teleop command
2. **Pick Up Task** (optional): Data collector selects a task, transitions to "assigned"
3. **Use Teleop Device**: Data collector uses teleop device (joystick, button, etc.)
   - Robot already has the configuration
   - Teleop just sends start/stop command
4. **Teleop Triggers Recording**: Teleop device sends command → Dagster op → Axon ROS service
   - Uses the config that was already pushed to robot
5. **Monitor Recording**: Data collector can monitor via UI
6. **Stop Recording**: Teleop device can also trigger stop command

**Teleop Device Integration**:

```python
# Teleop device can be:
# 1. Physical device (joystick, button, etc.) → ROS topic/service
# 2. Mobile app → REST API → Dagster
# 3. Web UI button → Dagster API

# Example: ROS topic from teleop device
# Topic: /teleop/recording_commands
# Message: {task_id: "xxx", command: "start" or "stop"}
```

**Teleop Command Handler**:

```python
@op
def teleop_command_handler_op(context, teleop_command: Dict[str, Any]) -> Dict[str, Any]:
    """
    Handle commands from teleop device.
    """
    task_id = teleop_command["task_id"]
    command = teleop_command["command"]  # "start" or "stop"
    
    if command == "start":
        return teleop_start_recording_op(context, task_id)
    elif command == "stop":
        return teleop_stop_recording_op(context, task_id)
```

---

## 9. Data Flow

### 9.1 Data Flow Overview

The data flow is triggered by task execution and follows this path:

```
Task Execution
    ↓
Start Recording (Axon via ROS)
    ↓
Data Recording (ROS Messages → Lance Dataset)
    ↓
Edge Cache Storage (Local/Edge Storage)
    ↓
Inspection Trigger
    ├── Automated Inspection (Daft/Ray)
    └── Manual Inspection (Human)
    ↓
Inspection Result
    ├── Approved → Cloud Upload
    └── Rejected → Archive/Delete
```

### 9.2 Recording Operations

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "task_id": Field(str, is_required=True, description="Task ID"),
        "config_path": Field(str, is_required=False, description="Path to YAML config file"),
        "dataset_path": Field(str, is_required=False, description="Override dataset path"),
        "topics": Field(list, is_required=False, description="List of topics to record"),
        "scene": Field(str, is_required=False, description="Scene name for metadata"),
        "sub_scene": Field(str, is_required=False, description="Sub-scene name for metadata"),
        "skills": Field(list, is_required=False, description="Skills for metadata"),
    },
    required_resource_keys={"axon_ros_client"},
)
def start_recording_op(context, task_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Start Axon recording session for a task.
    Includes scene/sub-scene/skills metadata in dataset path.
    """
    task_id = task_data["task_id"]
    scene = task_data.get("scene", context.op_config.get("scene", "unknown"))
    sub_scene = task_data.get("sub_scene", context.op_config.get("sub_scene", ""))
    skills = task_data.get("skills", context.op_config.get("skills", []))
    
    # Generate dataset path with metadata
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    skills_str = "_".join(skills) if skills else "all"
    sub_scene_str = f"_{sub_scene}" if sub_scene else ""
    dataset_path = f"/data/edge_cache/{scene}{sub_scene_str}_{skills_str}_{timestamp}.lance"
    
    # Create config with metadata
    config = {
        "dataset": {
            "path": dataset_path,
            "mode": "create",
        },
        "topics": context.op_config.get("topics", []),
        "metadata": {
            "task_id": task_id,
            "scene": scene,
            "sub_scene": sub_scene,
            "skills": skills,
            "start_time": datetime.now().isoformat(),
        },
    }
    
    # Save config temporarily
    config_path = f"/tmp/axon_config_{task_id}.yaml"
    with open(config_path, "w") as f:
        yaml.dump(config, f)
    
    # Start recording via ROS service
    client = context.resources.axon_ros_client
    response = client.start_recording(config_path=config_path)
    
    if not response.success:
        raise Failure(f"Failed to start recording: {response.message}")
    
    context.log.info(f"Started recording for task {task_id} at {dataset_path}")
    
    return {
        "task_id": task_id,
        "dataset_path": dataset_path,
        "recording_started": True,
        "config_path": config_path,
    }
```

### 9.3 Edge Cache Storage

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "source_path": Field(str, is_required=True, description="Source dataset path"),
        "edge_cache_path": Field(str, is_required=False, description="Edge cache destination path"),
    },
    required_resource_keys={"edge_storage"},
)
def move_to_edge_cache_op(context, recording_result: Dict[str, Any]) -> Dict[str, Any]:
    """
    Move recorded dataset to edge cache storage.
    Edge cache is the staging area before inspection and cloud upload.
    """
    source_path = recording_result["dataset_path"]
    edge_storage = context.resources.edge_storage
    
    # Generate edge cache path if not provided
    if not context.op_config.get("edge_cache_path"):
        # Extract metadata from source path or recording result
        task_id = recording_result["task_id"]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        edge_cache_path = f"{edge_storage.base_path}/edge_cache/{task_id}_{timestamp}.lance"
    else:
        edge_cache_path = context.op_config["edge_cache_path"]
    
    # Copy/move dataset to edge cache
    # This could be a simple file copy or a more sophisticated operation
    edge_storage.copy_to_edge_cache(source_path, edge_cache_path)
    
    context.log.info(f"Moved dataset to edge cache: {edge_cache_path}")
    
    return {
        "task_id": recording_result["task_id"],
        "source_path": source_path,
        "edge_cache_path": edge_cache_path,
        "size_bytes": edge_storage.get_size(edge_cache_path),
        "moved_at": datetime.now().isoformat(),
    }
```

### 9.4 Inspection Operations

**File**: `axon_dagster/ops.py`

#### 9.4.1 Automated Inspection (Daft on Ray)

**Architecture Overview**:
```
Dagster Op
    ↓
Daft Code (Dataframe operations)
    ↓
Ray Backend (Distributed execution)
    ↓
Ray Cluster (IP:Port)
```

**Key Points**:
- **Dagster** orchestrates and triggers the inspection op
- **Daft** is the dataframe library you write code with
- **Ray** is the execution backend that Daft uses for distributed processing
- You configure Daft to connect to Ray cluster via `ray_address` (IP:Port)

```python
@op(
    config_schema={
        "inspection_type": Field(str, default_value="daft", description="Inspection type: 'daft' (uses Ray) or 'daft-standalone'"),
        "ray_address": Field(str, default_value="auto", description="Ray cluster address (e.g., 'ray://head-node:10001' or 'auto' for local)"),
        "inspection_config": Field(dict, is_required=False, description="Inspection configuration"),
    },
    required_resource_keys={"edge_storage", "ray_cluster"},  # Ray resource optional
)
def automated_inspection_op(context, edge_cache_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Perform automated inspection using Daft.
    Daft can run standalone or on Ray cluster for distributed processing.
    """
    edge_cache_path = edge_cache_data["edge_cache_path"]
    inspection_type = context.op_config.get("inspection_type", "daft")
    ray_address = context.op_config.get("ray_address", "auto")
    inspection_config = context.op_config.get("inspection_config", {})
    
    import daft
    
    # Configure Daft to use Ray backend (if not standalone)
    if inspection_type == "daft" and ray_address != "auto":
        # Connect Daft to Ray cluster
        # Daft automatically uses Ray if Ray is initialized
        import ray
        if not ray.is_initialized():
            ray.init(address=ray_address)  # Connect to Ray cluster
        context.log.info(f"Using Ray cluster at {ray_address}")
    elif inspection_type == "daft-standalone":
        # Daft runs standalone (single machine, no Ray)
        context.log.info("Using Daft standalone (no Ray)")
    
    # Load dataset from edge cache using Daft
    # Daft will use Ray backend if Ray is initialized
    df = daft.read_lance(edge_cache_path)
    
    # Perform inspection checks (these run on Ray if Ray is initialized)
    checks = {
        "row_count": len(df),  # This triggers computation on Ray
        "schema_valid": True,
        "data_quality": "good",
        "completeness": 1.0,
    }
    
    # Run custom inspection logic
    if inspection_config.get("checks"):
        for check_name, check_func in inspection_config["checks"].items():
            checks[check_name] = check_func(df)  # Runs on Ray cluster
    
    approval_status = "approved" if all(checks.values()) else "needs_review"
    
    context.log.info(f"Inspection completed: {approval_status}")
    
    return {
        "task_id": edge_cache_data["task_id"],
        "edge_cache_path": edge_cache_path,
        "inspection_type": inspection_type,
        "ray_address": ray_address if inspection_type == "daft" else None,
        "checks": checks,
        "approval_status": approval_status,
        "inspected_at": datetime.now().isoformat(),
    }
```

**Alternative: Using Ray Resource**:

```python
@op(
    required_resource_keys={"edge_storage", "ray_cluster"},
)
def automated_inspection_op(context, edge_cache_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Using Ray cluster from Dagster resource.
    """
    # Ray is already initialized via resource
    ray_cluster = context.resources.ray_cluster  # Ray resource
    
    import daft
    
    # Daft automatically uses Ray since Ray is initialized
    df = daft.read_lance(edge_cache_data["edge_cache_path"])
    
    # All Daft operations run on Ray cluster
    checks = perform_inspection_checks(df)
    
    return {"approval_status": "approved" if checks else "needs_review"}
```

#### 9.4.2 Manual Inspection (Human)

```python
@op(
    config_schema={
        "wait_for_approval": Field(bool, default_value=True, description="Wait for human approval"),
        "approval_timeout_seconds": Field(float, default_value=86400.0, description="Approval timeout (24h default)"),
    },
    required_resource_keys={"edge_storage"},
)
def manual_inspection_op(context, edge_cache_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Trigger manual inspection workflow.
    Creates a pending inspection request that waits for human approval.
    """
    edge_cache_path = edge_cache_data["edge_cache_path"]
    task_id = edge_cache_data["task_id"]
    
    # Create inspection request
    inspection_request = {
        "task_id": task_id,
        "edge_cache_path": edge_cache_path,
        "status": "pending",
        "created_at": datetime.now().isoformat(),
        "metadata": edge_cache_data.get("metadata", {}),
    }
    
    # Store inspection request (database, file, or Dagster asset)
    # This would be accessible via Dagster UI for human review
    
    if context.op_config.get("wait_for_approval", True):
        # Wait for approval (polling or event-based)
        timeout = context.op_config.get("approval_timeout_seconds", 86400.0)
        approval = wait_for_approval(inspection_request, timeout)
        
        return {
            "task_id": task_id,
            "edge_cache_path": edge_cache_path,
            "inspection_type": "manual",
            "approval_status": approval["status"],  # "approved" or "rejected"
            "approved_by": approval.get("approved_by"),
            "approved_at": approval.get("approved_at"),
        }
    else:
        # Don't wait, just create the request
        return {
            "task_id": task_id,
            "edge_cache_path": edge_cache_path,
            "inspection_type": "manual",
            "approval_status": "pending",
            "inspection_request_id": inspection_request["request_id"],
        }
```

### 9.5 Cloud Upload

**File**: `axon_dagster/ops.py`

```python
@op(
    config_schema={
        "cloud_destination": Field(str, is_required=True, description="Cloud storage destination (S3, GCS, etc.)"),
        "delete_after_upload": Field(bool, default_value=False, description="Delete from edge cache after upload"),
    },
    required_resource_keys={"edge_storage", "cloud_storage"},
)
def cloud_upload_op(context, inspection_result: Dict[str, Any]) -> Dict[str, Any]:
    """
    Upload dataset to cloud storage after inspection approval.
    Only executes if inspection_status is "approved".
    """
    if inspection_result["approval_status"] != "approved":
        raise Failure(f"Cannot upload: inspection status is {inspection_result['approval_status']}")
    
    edge_cache_path = inspection_result["edge_cache_path"]
    task_id = inspection_result["task_id"]
    cloud_destination = context.op_config["cloud_destination"]
    
    # Generate cloud path with metadata
    scene = inspection_result.get("metadata", {}).get("scene", "unknown")
    sub_scene = inspection_result.get("metadata", {}).get("sub_scene", "")
    skills = inspection_result.get("metadata", {}).get("skills", [])
    skills_str = "_".join(skills) if skills else "all"
    sub_scene_str = f"_{sub_scene}" if sub_scene else ""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    cloud_path = f"{cloud_destination}/{scene}{sub_scene_str}_{skills_str}_{timestamp}.lance"
    
    # Upload to cloud
    cloud_storage = context.resources.cloud_storage
    cloud_storage.upload(edge_cache_path, cloud_path)
    
    context.log.info(f"Uploaded dataset to cloud: {cloud_path}")
    
    # Optionally delete from edge cache
    if context.op_config.get("delete_after_upload", False):
        edge_storage = context.resources.edge_storage
        edge_storage.delete(edge_cache_path)
        context.log.info(f"Deleted from edge cache: {edge_cache_path}")
    
    return {
        "task_id": task_id,
        "edge_cache_path": edge_cache_path,
        "cloud_path": cloud_path,
        "uploaded_at": datetime.now().isoformat(),
        "size_bytes": cloud_storage.get_size(cloud_path),
    }
```

### 9.6 Complete Data Flow Job

**File**: `axon_dagster/jobs.py`

```python
@job(
    resource_defs={
        "axon_ros_client": axon_ros_client,
        "edge_storage": edge_storage_resource,
        "cloud_storage": cloud_storage_resource,
    },
)
def data_flow_job():
    """
    Complete data flow: Recording → Edge Cache → Inspection → Cloud Upload
    """
    # This job is triggered by task execution
    
    # Step 1: Start recording
    recording = start_recording_op()
    
    # Step 2: Wait for recording (monitor and wait for duration)
    wait_result = wait_for_recording_op(recording)
    
    # Step 3: Stop recording
    stop_result = stop_recording_op(wait_result)
    
    # Step 4: Move to edge cache
    edge_cache = move_to_edge_cache_op(stop_result)
    
    # Step 5: Inspection (automated or manual)
    # This could be conditional based on configuration
    inspection = automated_inspection_op(edge_cache)
    # OR
    # inspection = manual_inspection_op(edge_cache)
    
    # Step 6: Cloud upload (only if approved)
    cloud_upload = cloud_upload_op(inspection)
    
    return cloud_upload
```

### 9.7 Data Flow State Transitions

```
Recording: not_started → recording → stopped
Edge Cache: not_cached → cached
Inspection: pending → in_progress → approved/rejected
Cloud Upload: not_uploaded → uploading → uploaded
```

```
┌─────────────────────────────────────────────────────────────┐
│  Dagster Scheduler/Sensor                                    │
│  Triggers: daily_recording_schedule                         │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Dagster Run Execution                                       │
│  1. Create RunRequest                                        │
│  2. Initialize Resources (AxonROSClient)                    │
│  3. Execute Ops                                              │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  start_recording_op                                          │
│  → AxonROSClient.start_recording()                          │
│  → ROS Service Call: /axon/start_recording                  │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Axon Recorder Node                                          │
│  1. Receive service request                                  │
│  2. Load configuration                                       │
│  3. Initialize batch managers                                │
│  4. Start subscriptions                                      │
│  5. Begin recording                                          │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  wait_for_recording_op                                       │
│  → Poll get_status_op periodically                          │
│  → Monitor recording health                                 │
│  → Wait for duration_seconds                                │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  stop_recording_op                                           │
│  → AxonROSClient.stop_recording()                          │
│  → ROS Service Call: /axon/stop_recording                  │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Axon Recorder Node                                          │
│  1. Stop subscriptions                                       │
│  2. Flush all batch managers                                │
│  3. Write final batches to Lance                            │
│  4. Return success                                          │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  lance_dataset_asset                                         │
│  → Verify dataset exists                                    │
│  → Extract metadata                                         │
│  → Register as Dagster asset                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 10. Asset Management

### 10.1 Hierarchical Asset Structure

Assets in Dagster represent the data artifacts and their relationships:

```
Order Asset
  ├── Metadata: scene, sub-scene, skills, order_id
  ├── Status: pending, active, completed
  └── Batch Assets (children)
      ├── Batch Asset 1
      │   ├── Metadata: scene, sub-scene, skills (inherited)
      │   ├── Status: pending, ready, executing, completed
      │   └── Task Assets (children)
      │       ├── Task Asset 1
      │       │   ├── Metadata: scene, sub-scene, skills (inherited)
      │       │   ├── Status: pending, executing, completed
      │       │   └── Dataset Asset (child)
      │       │       ├── Metadata: scene, sub-scene, skills, task_id
      │       │       ├── Location: edge_cache_path, cloud_path
      │       │       └── Status: recording, cached, inspected, uploaded
      │       └── Task Asset N
      └── Batch Asset N
```

### 10.2 Asset Definitions

**File**: `axon_dagster/assets.py`

```python
@asset(
    config_schema={
        "order_id": Field(str, is_required=True),
        "scene": Field(str, is_required=True),
        "sub_scene": Field(str, is_required=False),
        "skills": Field(list, is_required=True),
    },
)
def order_asset(context) -> Dict[str, Any]:
    """Asset representing a data collection order."""
    return {
        "order_id": context.op_config["order_id"],
        "scene": context.op_config["scene"],
        "sub_scene": context.op_config.get("sub_scene", ""),
        "skills": context.op_config["skills"],
        "status": "pending",
    }

@asset(
    config_schema={
        "batch_id": Field(str, is_required=True),
        "order_id": Field(str, is_required=True),
        "scene": Field(str, is_required=True),
        "sub_scene": Field(str, is_required=False),
        "skills": Field(list, is_required=True),
    },
)
def batch_asset(context, order_asset: Dict[str, Any]) -> Dict[str, Any]:
    """Asset representing a batch within an order."""
    return {
        "batch_id": context.op_config["batch_id"],
        "order_id": context.op_config["order_id"],
        "scene": context.op_config["scene"],
        "sub_scene": context.op_config.get("sub_scene", ""),
        "skills": context.op_config["skills"],
        "status": "pending",
    }

@asset(
    config_schema={
        "task_id": Field(str, is_required=True),
        "batch_id": Field(str, is_required=True),
        "scene": Field(str, is_required=True),
        "sub_scene": Field(str, is_required=False),
        "skills": Field(list, is_required=True),
    },
)
def task_asset(context, batch_asset: Dict[str, Any]) -> Dict[str, Any]:
    """Asset representing a task within a batch."""
    return {
        "task_id": context.op_config["task_id"],
        "batch_id": context.op_config["batch_id"],
        "scene": context.op_config["scene"],
        "sub_scene": context.op_config.get("sub_scene", ""),
        "skills": context.op_config["skills"],
        "status": "pending",
    }

@asset(
    config_schema={
        "dataset_path": Field(str, is_required=True),
        "task_id": Field(str, is_required=True),
        "scene": Field(str, is_required=True),
        "sub_scene": Field(str, is_required=False),
        "skills": Field(list, is_required=True),
    },
    required_resource_keys={"edge_storage"},
)
def dataset_asset(context, task_asset: Dict[str, Any]) -> Dict[str, Any]:
    """
    Asset representing a Lance dataset.
    Includes scene, sub-scene, and skills metadata.
    """
    dataset_path = context.op_config["dataset_path"]
    edge_storage = context.resources.edge_storage
    
    # Verify dataset exists
    if not edge_storage.exists(dataset_path):
        raise Failure(f"Dataset not found: {dataset_path}")
    
    # Extract metadata
    metadata = {
        "dataset_path": dataset_path,
        "task_id": context.op_config["task_id"],
        "scene": context.op_config["scene"],
        "sub_scene": context.op_config.get("sub_scene", ""),
        "skills": context.op_config["skills"],
        "size_bytes": edge_storage.get_size(dataset_path),
        "created_at": datetime.now().isoformat(),
    }
    
    return metadata
```

### 10.3 Asset Lineage

Dagster automatically tracks asset lineage. The hierarchical structure enables:

- **Upstream Dependencies**: See which order/batch/task a dataset belongs to
- **Downstream Dependencies**: See all datasets created from an order
- **Metadata Propagation**: Scene, sub-scene, skills flow from order → batch → task → dataset
- **Status Tracking**: Track status at each level

### 10.4 Asset Querying

Assets can be queried by metadata:

```python
# Query all datasets for a specific scene
scene_datasets = query_assets(scene="warehouse")

# Query all datasets for a specific skill
skill_datasets = query_assets(skills=["navigation"])

# Query all datasets for a scene and sub-scene
scene_subscene_datasets = query_assets(scene="warehouse", sub_scene="aisle_1")
```

---

## 11. Human-in-the-Loop

### 11.1 Human Workflow Overview

The system supports human-in-the-loop workflows at multiple stages:

1. **Order Creation**: Admin creates orders via Dagster UI
2. **Batch Generation**: Admin can manually generate or approve batches
3. **Manual Inspection**: Human reviewers inspect datasets
4. **Approval/Rejection**: Human approval required for cloud upload

### 11.2 Order Creation UI

**Dagster UI Integration**:

```python
# Dagster UI provides forms for order creation
@op(
    config_schema={
        "order_id": Field(str, is_required=True),
        "scene": Field(str, is_required=True),
        # ... other fields
    },
)
def create_order_op(context):
    """Create order - can be triggered from UI form."""
    # Implementation
```

**UI Features**:
- Form-based order creation
- Scene/sub-scene/skills selection
- Requirements specification
- Priority setting
- Order preview before submission

### 11.3 Batch Generation UI

Admins can:
- View generated batches
- Modify batch configurations
- Approve/reject batches
- Manually create batches

### 11.4 Manual Inspection Workflow

**Inspection Request Creation**:

```python
@op
def create_inspection_request_op(context, edge_cache_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Create inspection request for human review.
    This creates a pending request visible in Dagster UI.
    """
    # Create inspection request
    # Store in database or Dagster asset
    # Make visible in UI
    pass
```

**Inspection UI**:
- List of pending inspection requests
- Dataset preview (metadata, size, location)
- Approve/Reject buttons
- Comments/notes field
- Inspection history

**Approval Handling**:

```python
@op
def handle_approval_op(context, inspection_request_id: str, approval: str) -> Dict[str, Any]:
    """
    Handle human approval/rejection of inspection request.
    """
    # Update inspection request status
    # Trigger cloud upload if approved
    # Archive if rejected
    pass
```

### 11.5 Human Workflow Integration

```python
@job
def human_inspection_job():
    """
    Job that waits for human approval before proceeding.
    """
    # Record data
    recording = start_recording_op()
    wait = wait_for_recording_op(recording)
    stop = stop_recording_op(wait)
    
    # Move to edge cache
    edge_cache = move_to_edge_cache_op(stop)
    
    # Create inspection request (human-triggered)
    inspection_request = create_inspection_request_op(edge_cache)
    
    # Wait for approval (human action via UI)
    approval = wait_for_approval_op(inspection_request)
    
    # Upload if approved
    cloud_upload = cloud_upload_op(approval)
    
    return cloud_upload
```

---

## 12. Storage Architecture

### 12.1 Storage Layers

The system uses a three-tier storage architecture:

```
┌─────────────────────────────────────────────────────────────┐
│              Recording Storage (Local)                       │
│  - Temporary storage during recording                        │
│  - Managed by Axon                                           │
│  - Path: /data/recordings/                                  │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Edge Cache Storage                               │
│  - Staging area before inspection                             │
│  - Local or edge server storage                              │
│  - Path: /data/edge_cache/                                  │
│  - Supports: local filesystem, NFS, S3-compatible           │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Inspection Storage                               │
│  - Temporary storage during inspection                        │
│  - May be same as edge cache or separate                     │
│  - Accessible by Daft/Ray/Human                              │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Cloud Storage                                    │
│  - Final destination after approval                           │
│  - Supports: S3, GCS, Azure Blob                            │
│  - Organized by scene/sub-scene/skills                       │
└─────────────────────────────────────────────────────────────┘
```

### 12.2 Edge Cache Storage

**Purpose**:
- Staging area before inspection
- Fast local access for inspection tools
- Temporary storage before cloud upload
- Enables data validation before cloud transfer

**Characteristics**:
- **Location**: Local filesystem or edge server
- **Capacity**: Configurable (default: 1TB)
- **Cleanup Policy**: LRU, FIFO, or manual
- **Access**: Read/write for inspection tools

**Implementation**:

```python
class EdgeStorage:
    def __init__(self, base_path: str, max_size_gb: float, cleanup_policy: str):
        self.base_path = base_path
        self.max_size_gb = max_size_gb
        self.cleanup_policy = cleanup_policy
    
    def copy_to_edge_cache(self, source_path: str, dest_path: str):
        """Copy dataset to edge cache."""
        # Implementation: copy file or dataset
        pass
    
    def cleanup(self):
        """Cleanup old datasets based on policy."""
        if self.cleanup_policy == "lru":
            # Remove least recently used
            pass
        elif self.cleanup_policy == "fifo":
            # Remove oldest first
            pass
```

### 12.3 Inspection Storage

**Purpose**:
- Provide access to datasets for inspection tools
- May be same location as edge cache or separate
- Optimized for read access by Daft/Ray

**Daft Integration (with Ray backend)**:

```python
# Daft can read directly from edge cache
# If Ray is initialized, Daft automatically uses Ray for distributed processing
import daft
import ray

# Connect to Ray cluster (IP:Port)
ray.init(address="ray://head-node:10001")  # Your Ray cluster address

# Daft automatically uses Ray backend if Ray is initialized
df = daft.read_lance("/data/edge_cache/dataset.lance")

# All operations run on Ray cluster automatically
checks = df.filter(...).select(...).collect()  # Distributed on Ray
```

**Key Understanding**:
- **Dagster** → triggers the inspection op
- **Daft** → the dataframe library you write code with
- **Ray** → the execution backend (Daft uses Ray automatically)
- **You only need**: Ray cluster IP:Port (e.g., `ray://192.168.1.100:10001`)

**How it works**:
1. Dagster op initializes Ray connection: `ray.init(address="ray://ip:port")`
2. Daft detects Ray is initialized
3. Daft automatically uses Ray for all operations
4. No code changes needed - Daft handles distribution transparently

**Alternative: Direct Ray (without Daft)**:

```python
# If you want to use Ray directly (not through Daft)
import ray

@ray.remote
def inspect_dataset(path: str):
    # Custom inspection logic
    pass

results = inspect_dataset.remote("/data/edge_cache/dataset.lance")
```

**Recommendation**: Use Daft on Ray (not direct Ray) because:
- Daft provides high-level dataframe API (easier to use)
- Daft handles Ray integration automatically
- Better for data inspection workflows

### 12.4 Cloud Storage

**Purpose**:
- Long-term storage after approval
- Organized by scene/sub-scene/skills
- Accessible for downstream processing

**Organization**:

```
s3://bucket/
  warehouse/
    aisle_1/
      navigation_20240101_120000.lance
      manipulation_20240101_130000.lance
    aisle_2/
      navigation_20240101_140000.lance
  outdoor/
    parking_lot/
      navigation_20240101_150000.lance
```

**Implementation**:

```python
class CloudStorage:
    def upload(self, local_path: str, cloud_path: str):
        """Upload dataset to cloud storage."""
        # Implementation using boto3, gcs, etc.
        pass
    
    def generate_path(self, scene: str, sub_scene: str, skills: List[str]) -> str:
        """Generate cloud path from metadata."""
        skills_str = "_".join(skills)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        sub_scene_str = f"/{sub_scene}" if sub_scene else ""
        return f"{scene}{sub_scene_str}/{skills_str}_{timestamp}.lance"
```

### 12.5 Storage Lifecycle

```
1. Recording → Local temporary storage (Axon)
2. Recording complete → Move to edge cache
3. Inspection → Read from edge cache
4. Approval → Upload to cloud
5. Optional: Delete from edge cache after upload
6. Optional: Archive old edge cache data
```

---

## 13. Configuration Management

### 8.1 Dagster Configuration

**File**: `axon_dagster/config_schema.py`

```python
# Resource configuration
AXON_RESOURCE_CONFIG = {
    "ros_version": "ros2",  # or "ros1"
    "namespace": "axon",
    "timeout_seconds": 30.0,
    "retry_attempts": 3,
}

# Op configuration templates
RECORDING_CONFIG_TEMPLATES = {
    "default": {
        "config_path": "/configs/default.yaml",
    },
    "camera_only": {
        "config_path": "/configs/camera_only.yaml",
    },
    "lidar_only": {
        "config_path": "/configs/lidar_only.yaml",
    },
}
```

### 8.2 Axon Configuration Files

Configuration files remain in YAML format as currently used by Axon:

```yaml
# /configs/default.yaml
dataset:
  path: "/data/recordings/dataset_{timestamp}.lance"
  mode: "create"

topics:
  - name: "/camera/image_raw"
    message_type: "sensor_msgs/Image"
    batch_size: 100
    flush_interval_ms: 1000

recording:
  auto_start: false  # Controlled by Dagster
  max_disk_usage_gb: 100.0
```

### 8.3 Environment-Specific Configuration

```python
# Development
dev_config = {
    "ros_version": "ros2",
    "namespace": "axon",
    "timeout_seconds": 10.0,
}

# Production
prod_config = {
    "ros_version": "ros2",
    "namespace": "axon",
    "timeout_seconds": 60.0,
    "retry_attempts": 5,
}
```

---

## 9. Error Handling & Resilience

### 9.1 Error Categories

1. **ROS Connection Errors**: ROS master/DDS not available
2. **Service Call Errors**: Service timeout, service not found
3. **Recording Errors**: Start/stop failures, configuration errors
4. **Resource Errors**: Disk full, memory issues
5. **Network Errors**: ROS network connectivity issues

### 9.2 Retry Strategy

```python
@op(
    retry_policy=RetryPolicy(
        max_retries=3,
        delay=5.0,
        backoff=Backoff.EXPONENTIAL,
    ),
)
def start_recording_op(context):
    """Start recording with retry logic."""
    # Implementation with retry
```

### 9.3 Error Handling in Ops

```python
def start_recording_op(context):
    try:
        client = context.resources.axon_ros_client
        response = client.start_recording(config_path)
        
        if not response.success:
            raise Failure(
                description=f"Failed to start recording: {response.message}",
                metadata={
                    "config_path": config_path,
                    "error_message": response.message,
                }
            )
        
        return {
            "success": True,
            "dataset_path": response.dataset_path,
            "timestamp": datetime.now().isoformat(),
        }
    except ROSException as e:
        raise Failure(
            description=f"ROS error: {str(e)}",
            metadata={"error_type": "ros_connection"},
        )
```

### 9.4 Health Checks

```python
@op
def health_check_op(context) -> Dict[str, Any]:
    """Check Axon and ROS system health."""
    client = context.resources.axon_ros_client
    
    # Check ROS connection
    if not client.is_connected():
        raise Failure("ROS not connected")
    
    # Check Axon status
    status = client.get_status()
    
    # Check disk space
    if status.disk_usage_gb > 90.0:
        context.log.warning(f"High disk usage: {status.disk_usage_gb}GB")
    
    return {
        "healthy": True,
        "disk_usage": status.disk_usage_gb,
        "is_recording": status.is_recording,
    }
```

---

## 10. Monitoring & Observability

### 10.1 Dagster UI Integration

The Dagster UI will provide:

- **Run History**: View all recording job runs
- **Asset Lineage**: Visualize dataset dependencies
- **Logs**: View op execution logs
- **Metrics**: Recording duration, dataset sizes, success rates
- **Status**: Real-time job and op status

### 10.2 Custom Metrics

```python
@op
def start_recording_op(context):
    """Start recording with metrics."""
    start_time = time.time()
    
    # ... recording logic ...
    
    # Log metrics
    context.log.info(f"Recording started in {time.time() - start_time:.2f}s")
    
    # Emit custom metrics (if using Dagster metrics)
    # context.instance.report_asset_materialization(...)
```

### 10.3 Logging

```python
# Structured logging in ops
context.log.info(
    "Starting recording session",
    extra={
        "config_path": config_path,
        "dataset_path": dataset_path,
        "topics": topics,
    }
)
```

### 10.4 Alerting

```python
@sensor
def recording_failure_sensor(context):
    """Alert on recording failures."""
    # Check recent runs for failures
    # Send alerts if failure rate exceeds threshold
    pass
```

---

## 11. Implementation Phases

### Phase 1: Foundation (Weeks 1-2)

**Goals**: Basic ROS client and simple ops

- [ ] Create Python ROS client library (`ros_client.py`)
  - ROS1 support (rospy)
  - ROS2 support (rclpy)
  - Unified interface
  - Error handling
- [ ] Implement Dagster resource (`resources.py`)
  - AxonROSClient resource
  - Configuration schema
  - Resource initialization
- [ ] Implement basic ops (`ops.py`)
  - `start_recording_op`
  - `stop_recording_op`
  - `get_status_op`
- [ ] Unit tests for ROS client
- [ ] Integration tests with mock ROS services

**Deliverables**:
- Working ROS client library
- Basic Dagster ops
- Test suite

### Phase 2: Core Functionality (Weeks 3-4)

**Goals**: Complete ops and simple jobs

- [ ] Complete all ops
  - `update_config_op`
  - `wait_for_recording_op`
  - `health_check_op`
- [ ] Implement simple recording job
- [ ] Implement dataset asset
- [ ] Add retry logic and error handling
- [ ] Documentation

**Deliverables**:
- Complete op set
- Working simple recording job
- Asset tracking
- Documentation

### Phase 3: Orchestration (Weeks 5-6)

**Goals**: Multi-session and conditional workflows

- [ ] Multi-session recording job
- [ ] Conditional recording job
- [ ] Dependency management between sessions
- [ ] Configuration management improvements
- [ ] Enhanced error handling

**Deliverables**:
- Multi-session job
- Conditional workflows
- Configuration system

### Phase 4: Scheduling & Automation (Weeks 7-8)

**Goals**: Schedules and sensors

- [ ] Daily recording schedule
- [ ] Hourly recording schedule
- [ ] Disk space sensor
- [ ] Topic availability sensor
- [ ] Custom sensor framework

**Deliverables**:
- Scheduled jobs
- Event-driven sensors
- Automation framework

### Phase 5: Monitoring & Polish (Weeks 9-10)

**Goals**: Observability and production readiness

- [ ] Enhanced logging
- [ ] Custom metrics
- [ ] Alerting integration
- [ ] Performance optimization
- [ ] Comprehensive documentation
- [ ] Production deployment guide

**Deliverables**:
- Production-ready system
- Complete documentation
- Deployment guide

---

## 12. Testing Strategy

### 12.1 Unit Tests

**ROS Client Tests**:
- Mock ROS services
- Test service call success/failure
- Test timeout handling
- Test retry logic

**Op Tests**:
- Test op execution with mock resources
- Test configuration validation
- Test error handling

### 12.2 Integration Tests

**With Mock ROS Services**:
- Test full job execution
- Test multi-session workflows
- Test error recovery

**With Real Axon (Docker)**:
- Test against real Axon recorder
- Test ROS1 and ROS2 compatibility
- Test end-to-end workflows

### 12.3 End-to-End Tests

**Full Pipeline Tests**:
- Start Dagster
- Execute recording job
- Verify dataset creation
- Verify asset registration

### 12.4 Test Infrastructure

```python
# test/fixtures/mock_ros_services.py
class MockAxonROS1Service:
    """Mock ROS1 service for testing."""
    def __init__(self):
        self.recording = False
        self.status = {...}
    
    def start_recording(self, req):
        # Mock implementation
        pass
```

---

## 13. Code Organization

### 13.1 Repository Structure

Given that:
- **Axon** is a device agent (C++/Rust) deployed on robots
- **Dagster Integration** is a Python orchestration layer running on a central server
- They communicate via ROS services (no direct code dependencies)
- Different deployment targets and lifecycles

**Recommended Approach: Monorepo with Clear Separation**

```
axon/
├── README.md
├── ARCHITECTURE.md
├── Cargo.toml                    # Rust workspace
├── CMakeLists.txt                # C++ build
├── package.xml                    # ROS package
│
├── src/                          # Axon Core (Device Agent)
│   ├── bridge/                   # Rust FFI bridge
│   │   ├── Cargo.toml
│   │   └── src/
│   ├── cpp/                      # C++ core
│   │   ├── common/
│   │   ├── core/
│   │   ├── ffi/
│   │   ├── ros1/
│   │   └── ros2/
│   └── ...
│
├── dagster/                      # Dagster Integration (Orchestration)
│   ├── README.md
│   ├── pyproject.toml            # Python package
│   ├── setup.py
│   ├── axon_dagster/             # Main package
│   │   ├── __init__.py
│   │   ├── assets.py
│   │   ├── ops.py
│   │   ├── resources.py
│   │   ├── jobs.py
│   │   ├── schedules.py
│   │   ├── sensors.py
│   │   ├── config_schema.py
│   │   └── ros_client.py
│   ├── tests/
│   │   ├── unit/
│   │   ├── integration/
│   │   └── fixtures/
│   ├── configs/                  # Dagster-specific configs
│   │   ├── dagster.yaml
│   │   └── ...
│   └── docker/
│       └── Dockerfile.dagster
│
├── config/                       # Shared configs (Axon)
│   ├── default_config.yaml
│   └── message_schemas.yaml
│
├── launch/                       # ROS launch files (Axon)
│   ├── recorder_ros1.launch
│   └── recorder_ros2.launch.py
│
├── docker/                       # Docker for Axon
│   ├── Dockerfile.ros1
│   ├── Dockerfile.ros2.*
│   └── docker-compose.yml
│
├── test/                         # Axon tests
│   └── ...
│
└── docs/                         # Documentation
    ├── DAGSTER_INTEGRATION_DESIGN.md
    └── ...
```

### 13.2 Why Monorepo?

**Advantages**:
1. **Unified Versioning**: Axon and Dagster integration can be versioned together
2. **Shared Documentation**: All docs in one place
3. **Easier Development**: Developers can work on both components
4. **Unified CI/CD**: Single pipeline for testing both components
5. **Atomic Changes**: Changes to Axon services and Dagster integration can be coordinated
6. **Shared Configs**: Common configuration patterns can be shared

**Considerations**:
- Clear separation between `src/` (Axon) and `dagster/` (Dagster integration)
- Independent build systems (Cargo/CMake for Axon, Python for Dagster)
- Different deployment targets (robots vs server)

### 13.3 Package Structure Details

#### 13.3.1 Axon Package (Device Agent)

```
src/
├── bridge/                       # Rust FFI
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs
│       ├── writer.rs
│       └── error.rs
│
├── cpp/                          # C++ Core
│   ├── common/                  # Shared utilities
│   ├── core/                    # Core functionality
│   │   ├── batch_manager.cpp
│   │   ├── message_converter.cpp
│   │   └── ...
│   ├── ffi/                     # FFI bridge
│   ├── ros1/                    # ROS1 implementation
│   └── ros2/                    # ROS2 implementation
│
└── ...
```

**Build System**: 
- Rust: `cargo build` in `src/bridge/`
- C++: CMake in root directory
- ROS: `catkin_make` or `colcon build`

#### 13.3.2 Dagster Package (Orchestration)

```
dagster/
├── pyproject.toml               # Modern Python packaging
├── setup.py                      # Alternative setup
├── requirements.txt              # Python dependencies
│
├── axon_dagster/                # Main Python package
│   ├── __init__.py
│   ├── assets.py                # Asset definitions
│   ├── ops/                     # Operations (organized by domain)
│   │   ├── __init__.py
│   │   ├── control_flow.py      # Order/Batch/Task ops
│   │   ├── data_flow.py         # Recording/Inspection ops
│   │   └── storage.py           # Storage ops
│   ├── resources.py             # Resource definitions
│   ├── jobs.py                  # Job definitions
│   ├── schedules.py            # Schedule definitions
│   ├── sensors.py              # Sensor definitions
│   ├── config_schema.py        # Configuration schemas
│   └── clients/                # Client libraries
│       ├── __init__.py
│       ├── ros_client.py        # ROS service client
│       └── storage_client.py    # Storage clients
│
├── tests/
│   ├── unit/
│   │   ├── test_ros_client.py
│   │   ├── test_ops.py
│   │   └── test_resources.py
│   ├── integration/
│   │   ├── test_jobs.py
│   │   └── test_with_axon.py
│   └── fixtures/
│       └── mock_ros_services.py
│
├── configs/                     # Dagster configurations
│   ├── dagster.yaml
│   ├── dev.yaml
│   └── prod.yaml
│
└── docker/
    └── Dockerfile.dagster
```

**Build System**: 
- Python: `pip install -e .` or `poetry install`
- Dagster: `dagster dev` or `dagster-daemon run`

### 13.4 Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Development/CI Environment                     │
│                                                             │
│  Monorepo (Git)                                             │
│  ├── src/          → Build Axon binaries                    │
│  └── dagster/      → Build Python package                  │
└─────────────────────────────────────────────────────────────┘
                         ↓
         ┌───────────────┴───────────────┐
         ↓                                 ↓
┌─────────────────────┐      ┌──────────────────────────┐
│   Robot Deployment   │      │   Server Deployment       │
│                      │      │                          │
│  Axon Binary         │      │  Dagster Server          │
│  (C++/Rust)         │      │  (Python)                │
│                      │      │                          │
│  - ROS Node         │      │  - Orchestration         │
│  - ROS Services     │      │  - UI                    │
│  - Local Storage    │      │  - Storage Management    │
│                      │      │                          │
│  Deployed via:       │      │  Deployed via:           │
│  - Docker            │      │  - Docker                │
│  - ROS package       │      │  - Python package       │
│  - Systemd service   │      │  - Kubernetes            │
└─────────────────────┘      └──────────────────────────┘
         │                                 │
         └───────────────┬───────────────┘
                         ↓
              ┌──────────────────────┐
              │   ROS Network        │
              │   (Communication)    │
              └──────────────────────┘
```

### 13.5 Dependency Management

#### 13.5.1 Axon Dependencies

**Rust** (`src/bridge/Cargo.toml`):
```toml
[package]
name = "lance_writer_bridge"
version = "0.1.0"

[dependencies]
lance = "0.4"
arrow = "50.0"
```

**C++** (`CMakeLists.txt`):
```cmake
find_package(Arrow REQUIRED)
find_package(yaml-cpp REQUIRED)
# ROS dependencies handled by ROS build system
```

#### 13.5.2 Dagster Dependencies

**Python** (`dagster/pyproject.toml` or `requirements.txt`):

**Base Dependencies**:
```toml
[project]
name = "axon-dagster"
version = "0.1.0"
dependencies = [
    "dagster>=1.5.0",
    "dagster-webserver>=1.5.0",
    "rospy>=1.15.0; sys_platform != 'win32'",  # ROS1
    "rclpy>=3.0.0; sys_platform != 'win32'",   # ROS2
    "pyyaml>=6.0",
    "boto3>=1.28.0",  # AWS S3
    "google-cloud-storage>=2.10.0",  # GCS
]
```

**Optional Dependencies** (for inspection):
```toml
[project.optional-dependencies]
inspection = [
    "daft>=0.2.0",  # For Daft-based inspection
    "ray>=2.8.0",   # For Ray-based distributed inspection
]

# Install with: pip install axon-dagster[inspection]
```

**Alternative: Separate Requirements Files**:
```bash
# dagster/requirements.txt (base)
dagster>=1.5.0
dagster-webserver>=1.5.0
rospy>=1.15.0
rclpy>=3.0.0
pyyaml>=6.0
boto3>=1.28.0
google-cloud-storage>=2.10.0

# dagster/requirements-inspection.txt (optional)
-r requirements.txt
daft>=0.2.0
ray>=2.8.0
```

#### 13.5.3 Daft and Ray Integration

**Important**: Daft and Ray are **Python dependencies**, not separate projects. They fit naturally into the monorepo structure:

1. **Daft**: Python library for dataframes
   - Used within Dagster ops for inspection
   - No special infrastructure needed
   - Just a dependency in `dagster/requirements.txt`

2. **Ray**: Distributed computing framework
   - Used for distributed inspection operations
   - May require Ray cluster setup (but still just a dependency)
   - Configuration handled in Dagster resources/ops

**Code Organization with Daft/Ray**:
```
dagster/
├── axon_dagster/
│   ├── ops/
│   │   ├── data_flow.py        # Contains inspection ops
│   │   └── ...
│   ├── resources.py            # May include Ray cluster resource
│   └── ...
├── requirements.txt            # Base dependencies
├── requirements-inspection.txt # Daft + Ray (optional)
└── ...
```

**Ray Cluster Considerations**:
- Ray clusters are runtime infrastructure, not code
- Configuration in Dagster resources or environment
- No impact on monorepo structure
- Can be deployed separately (Kubernetes, standalone cluster)

**Example Ray Resource**:
```python
# dagster/axon_dagster/resources.py
@resource(
    config_schema={
        "ray_address": Field(
            str, 
            default_value="auto", 
            description="Ray cluster address. Format: 'ray://head-node-ip:port' or 'auto' for local"
        ),
        "num_cpus": Field(int, default_value=4, description="Number of CPUs for Ray (only for local)"),
    }
)
def ray_cluster_resource(context):
    """
    Resource for Ray cluster connection.
    
    Usage:
    - Local Ray: ray_address="auto" (starts local Ray)
    - Remote Ray: ray_address="ray://192.168.1.100:10001" (connects to cluster)
    """
    import ray
    
    ray_address = context.resource_config["ray_address"]
    
    if ray_address == "auto":
        # Start local Ray cluster
        ray.init(num_cpus=context.resource_config.get("num_cpus", 4))
        context.log.info("Initialized local Ray cluster")
    else:
        # Connect to remote Ray cluster
        ray.init(address=ray_address)
        context.log.info(f"Connected to Ray cluster at {ray_address}")
    
    return ray
```

**Dagster → Daft → Ray Flow**:

```
┌─────────────────────────────────────────────────────────────┐
│  Dagster Op Execution                                        │
│  @op def automated_inspection_op(...)                       │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Daft Code (Your Inspection Logic)                          │
│  import daft                                                 │
│  df = daft.read_lance(path)                                 │
│  checks = df.filter(...).select(...)                        │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Ray Backend (Automatic)                                     │
│  - Daft detects Ray is initialized                          │
│  - Automatically uses Ray for distributed execution         │
│  - No code changes needed                                   │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│  Ray Cluster (IP:Port)                                       │
│  ray://head-node:10001                                       │
│  - Executes Daft operations in parallel                     │
│  - Distributes work across nodes                            │
└─────────────────────────────────────────────────────────────┘
```

**Configuration Options**:

1. **Via Op Config** (per-op):
```python
@op(config_schema={
    "ray_address": Field(str, default_value="ray://192.168.1.100:10001")
})
def inspection_op(context):
    ray.init(address=context.op_config["ray_address"])
    # ... Daft code
```

2. **Via Resource** (shared across ops):
```python
# In resources.py
@resource(config_schema={"ray_address": Field(str)})
def ray_cluster(context):
    ray.init(address=context.resource_config["ray_address"])
    return ray

# In ops
@op(required_resource_keys={"ray_cluster"})
def inspection_op(context):
    # Ray already initialized via resource
    # Daft automatically uses it
    df = daft.read_lance(...)
```

3. **Via Environment Variable**:
```bash
export RAY_ADDRESS="ray://192.168.1.100:10001"
# Ray will use this automatically
```

### 13.6 Build and Release Process

#### 13.6.1 Development Workflow

```bash
# Clone monorepo
git clone <repo-url>
cd axon

# Build Axon (for robot deployment)
cd src/bridge && cargo build --release
cd ../.. && mkdir build && cd build && cmake .. && make

# Build Dagster integration (for server deployment)
cd dagster
pip install -e .  # or poetry install
```

#### 13.6.2 CI/CD Pipeline

```yaml
# .github/workflows/ci.yml (example)
name: CI

on: [push, pull_request]

jobs:
  test-axon:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Build and test Axon
        run: |
          cd src/bridge && cargo test
          cd ../.. && mkdir build && cd build && cmake .. && make test

  test-dagster:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Setup Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: |
          cd dagster
          pip install -e ".[dev]"
      - name: Run tests
        run: |
          cd dagster
          pytest tests/
```

#### 13.6.3 Release Process

1. **Version Bumping**: Update versions in both packages
   - Axon: `Cargo.toml`, `CMakeLists.txt`
   - Dagster: `pyproject.toml`

2. **Build Artifacts**:
   - Axon: Binary packages, Docker images
   - Dagster: Python wheel, Docker image

3. **Deployment**:
   - Axon: Deploy to robots (Docker, ROS package, etc.)
   - Dagster: Deploy to server (Docker, Kubernetes, etc.)

### 13.6 Daft and Ray: Impact on Monorepo Decision

**Short Answer: ✅ Yes, still use monorepo. Daft and Ray are Python dependencies, not separate projects, so they don't change the monorepo recommendation.**

**Key Point**: Daft and Ray are libraries you **import and use**, not codebases you **maintain**. They're dependencies in `requirements.txt`, similar to `dagster` or `boto3`.

#### 13.6.1 Why Daft/Ray Don't Change the Recommendation

1. **They're Dependencies, Not Projects**: Daft and Ray are Python libraries installed via pip, not separate codebases to maintain
2. **Used Within Dagster Integration**: They're used in Dagster ops for inspection, not as standalone services
3. **No Additional Code Organization**: They're just imports in Python code
4. **Optional Dependencies**: Can be installed optionally via `pip install axon-dagster[inspection]`

#### 13.6.2 How They Fit in Monorepo

```
axon/                          # Monorepo
├── src/                       # Axon (unchanged)
│   └── ...
│
├── dagster/                   # Dagster integration
│   ├── axon_dagster/
│   │   ├── ops/
│   │   │   └── data_flow.py   # Uses: import daft, import ray
│   │   └── resources.py       # May have Ray cluster resource
│   ├── requirements.txt       # Base deps
│   └── requirements-inspection.txt  # Daft + Ray (optional)
│
└── ...
```

**No structural changes needed** - just add dependencies to `requirements.txt`.

#### 13.6.3 Special Considerations

**Ray Clusters** (if using distributed Ray):
- Ray clusters are **runtime infrastructure**, not code
- Deployed separately (Kubernetes, standalone)
- Configuration via environment variables or Dagster resources
- **Doesn't affect monorepo structure**

**Daft**:
- Pure Python library
- No special infrastructure
- Just a dependency

**Example: Ray Cluster Deployment**:
```yaml
# Separate deployment config (not in monorepo)
# kubernetes/ray-cluster.yaml
apiVersion: ray.io/v1alpha1
kind: RayCluster
metadata:
  name: inspection-cluster
spec:
  # Ray cluster configuration
```

The Ray cluster YAML is deployment config, not source code, so it can live:
- In the monorepo under `deploy/` or `k8s/`
- In a separate infrastructure repo
- In deployment tooling (Helm charts, Terraform, etc.)

#### 13.6.4 When You Might Consider Separate Repos

**Only if you're building custom Daft/Ray extensions**:
- If you're contributing to Daft/Ray core
- If you're building a custom Ray operator
- If you're maintaining a fork of Daft

**But for typical usage** (importing and using libraries):
- ✅ Stay with monorepo
- ✅ Add as dependencies
- ✅ Use in Dagster ops

### 13.7 Alternative: Separate Repositories

If monorepo becomes too complex, consider:

```
axon/                    # Device agent repo
├── src/
└── ...

axon-dagster/           # Orchestration repo
├── axon_dagster/
└── ...
```

**When to use separate repos**:
- Very different release cycles
- Different teams with minimal coordination
- Need to open-source one but not the other
- Repository size becomes unwieldy
- **Custom Daft/Ray development** (not just usage)

**Coordination needed**:
- Version compatibility matrix
- Shared documentation
- Coordinated releases
- API/Service contract versioning

### 13.8 Recommended Structure Summary

**For this project, we recommend Monorepo because**:

1. ✅ **Tight Coupling**: Dagster integration depends on Axon ROS service interface
2. ✅ **Coordinated Development**: Changes to Axon services may require Dagster updates
3. ✅ **Unified Documentation**: Easier to maintain comprehensive docs
4. ✅ **Simplified CI/CD**: Single pipeline for both components
5. ✅ **Version Alignment**: Easier to keep versions in sync

**Key Principles**:
- Clear separation: `src/` for Axon, `dagster/` for orchestration
- Independent build systems: Each component builds independently
- Shared configs: Common patterns in root `config/`
- Unified docs: All documentation in `docs/`

---

## 14. Deployment Considerations

### 13.1 Deployment Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Dagster Server                                  │
│  (Docker container or host machine)                         │
│  - Dagster daemon                                            │
│  - Dagster UI                                                │
│  - Code location (axon_dagster/)                            │
└─────────────────────────────────────────────────────────────┘
                         ↕
┌─────────────────────────────────────────────────────────────┐
│              ROS Network                                     │
│  (Same network as Axon recorder)                            │
│  - ROS1 Master or ROS2 DDS                                  │
│  - Axon Recorder Node                                        │
└─────────────────────────────────────────────────────────────┘
```

### 13.2 Requirements

**Dagster Server**:
- Python 3.8+
- Dagster packages
- ROS Python libraries (rospy or rclpy)
- Network access to ROS network

**ROS Network**:
- ROS1: ROS master running
- ROS2: DDS discovery working
- Axon recorder node running
- Services accessible

### 13.3 Docker Deployment

```dockerfile
# docker/Dockerfile.dagster
FROM python:3.10-slim

# Install ROS Python libraries
# (ROS1 or ROS2 depending on target)

# Install Dagster
RUN pip install dagster dagster-webserver

# Install axon_dagster package
COPY axon_dagster/ /app/axon_dagster/
WORKDIR /app

# Run Dagster
CMD ["dagster", "dev", "-f", "axon_dagster/__init__.py"]
```

### 13.4 Configuration Management

**Environment Variables**:
```bash
DAGSTER_HOME=/var/lib/dagster
AXON_ROS_VERSION=ros2
AXON_NAMESPACE=axon
```

**Configuration Files**:
- Dagster configuration: `dagster.yaml`
- Axon configurations: `/configs/*.yaml`
- Job definitions: `axon_dagster/jobs.py`

### 13.5 Security Considerations

1. **ROS Network Security**: Ensure ROS network is properly secured
2. **Service Authentication**: Consider ROS service authentication if available
3. **File System Access**: Limit file system access for config files
4. **Network Isolation**: Use network isolation for production

---

## 15. Future Enhancements

### 14.1 Advanced Features

1. **Data Quality Checks**: Validate recorded data quality
2. **Automatic Schema Evolution**: Handle schema changes automatically
3. **Multi-Robot Support**: Orchestrate recording from multiple robots
4. **Real-time Monitoring**: Real-time dashboard for recording status
5. **Data Transformation**: Add data transformation ops
6. **Backup & Archival**: Automatic backup and archival workflows

### 14.2 Integration Enhancements

1. **ML Pipeline Integration**: Integrate with ML training pipelines
2. **Data Catalog Integration**: Integrate with data catalogs (e.g., Amundsen)
3. **Cloud Storage**: Support cloud storage backends
4. **Streaming**: Real-time streaming to external systems

### 14.3 Performance Optimizations

1. **Parallel Recording**: Record multiple topics in parallel
2. **Batch Optimization**: Optimize batch sizes based on metrics
3. **Resource Pooling**: Pool ROS connections
4. **Caching**: Cache configuration and status

---

## 16. Appendix

### 16.1 Monorepo File Structure

```
axon/                          # Root of monorepo
├── README.md
├── ARCHITECTURE.md
├── Cargo.toml                 # Rust workspace
├── CMakeLists.txt             # C++ build
├── package.xml                # ROS package
│
├── src/                       # Axon Core (Device Agent)
│   ├── bridge/               # Rust FFI bridge
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── writer.rs
│   │       └── error.rs
│   └── cpp/                  # C++ core
│       ├── common/
│       ├── core/
│       ├── ffi/
│       ├── ros1/
│       └── ros2/
│
├── dagster/                   # Dagster Integration (Orchestration)
│   ├── README.md
│   ├── pyproject.toml
│   ├── requirements.txt
│   ├── axon_dagster/         # Main Python package
│   │   ├── __init__.py
│   │   ├── assets.py
│   │   ├── ops/
│   │   │   ├── __init__.py
│   │   │   ├── control_flow.py
│   │   │   ├── data_flow.py
│   │   │   └── storage.py
│   │   ├── resources.py
│   │   ├── jobs.py
│   │   ├── schedules.py
│   │   ├── sensors.py
│   │   ├── config_schema.py
│   │   └── clients/
│   │       ├── __init__.py
│   │       ├── ros_client.py
│   │       └── storage_client.py
│   ├── tests/
│   │   ├── unit/
│   │   │   ├── test_ros_client.py
│   │   │   ├── test_ops.py
│   │   │   └── test_resources.py
│   │   ├── integration/
│   │   │   ├── test_jobs.py
│   │   │   └── test_with_axon.py
│   │   └── fixtures/
│   │       └── mock_ros_services.py
│   ├── configs/
│   │   ├── dagster.yaml
│   │   ├── dev.yaml
│   │   └── prod.yaml
│   └── docker/
│       └── Dockerfile.dagster
│
├── config/                    # Shared configs (Axon)
│   ├── default_config.yaml
│   └── message_schemas.yaml
│
├── launch/                    # ROS launch files (Axon)
│   ├── recorder_ros1.launch
│   └── recorder_ros2.launch.py
│
├── docker/                    # Docker for Axon
│   ├── Dockerfile.ros1
│   ├── Dockerfile.ros2.*
│   └── docker-compose.yml
│
├── test/                      # Axon tests
│   └── ...
│
└── docs/                      # Documentation
    ├── DAGSTER_INTEGRATION_DESIGN.md
    └── ...
```

### 15.2 Dependencies

**Python Packages**:
```
dagster>=1.5.0
dagster-webserver>=1.5.0
rospy>=1.15.0          # For ROS1
rclpy>=3.0.0           # For ROS2
pyyaml>=6.0
```

### 15.3 References

- [Dagster Documentation](https://docs.dagster.io/)
- [ROS1 Service Documentation](http://wiki.ros.org/Services)
- [ROS2 Service Documentation](https://docs.ros.org/en/humble/Tutorials/Services.html)
- [Axon Architecture Documentation](./ARCHITECTURE.md)

---

## 17. Conclusion

This design document outlines a comprehensive integration of Dagster with Axon, enabling powerful orchestration, scheduling, and monitoring capabilities for ROS data recording pipelines. The integration maintains separation of concerns by using ROS services as the interface, avoiding modifications to core Axon code.

The phased implementation approach allows for incremental development and testing, ensuring a robust and production-ready system. The design is extensible, allowing for future enhancements and integrations.

**Next Steps**:
1. Review and approve this design document
2. Set up development environment
3. Begin Phase 1 implementation
4. Establish testing infrastructure

---

**Document Status**: Ready for Review  
**Last Updated**: 2024  
**Version**: 1.0


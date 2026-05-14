# Metadata Injection Capability Design

**Date:** 2025-12-20
## Overview

This document defines the metadata injection capability for `axon_recorder`, enabling self-describing MCAP files for embodied robotics data recording. The design ensures that recorded data remains traceable and queryable even after file renaming or migration across storage systems.

## Problem Statement

Current file-based metadata approaches have limitations:

| Approach | Limitation |
|----------|------------|
| File naming convention | Length limits, easily truncated, can be renamed |
| External database lookup | Requires network connectivity, single point of failure |
| Directory structure | Brittle, breaks on reorganization |

**Solution**: Embed all contextual metadata directly into the MCAP file structure, making each file self-describing and independently queryable.

## Design Goals

1. **Self-Describing Files** - Each MCAP file contains all context needed to identify its origin and purpose
2. **Edge-First Processing** - Sidecar JSON enables fast metadata parsing without opening MCAP
3. **Traceable Data Lineage** - Complete audit trail from recording to cloud storage
4. **Schema Evolution** - Metadata format can evolve without breaking existing files
5. **Zero Runtime Overhead** - Metadata written once at file creation/finalization

## Architecture

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                      Metadata Injection Architecture                          │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│   ┌─────────────────┐                                                         │
│   │  Data Collector │ ─── CachedRecordingConfig ───►  Task Metadata           │
│   │    (Server)     │     (task_id, scene, skills, etc.)                      │
│   └─────────────────┘                                                         │
│                                                                               │
│                              ▼                                                │
│                                                                               │
│   ┌─────────────────────────────────────────────────────────────────────┐    │
│   │                        axon_recorder                                 │    │
│   │  ┌─────────────────────────────────────────────────────────────┐    │    │
│   │  │                    Recording Session                         │    │    │
│   │  │                                                              │    │    │
│   │  │   Task Metadata ──► MCAP Header/Metadata Section            │    │    │
│   │  │        │                      │                              │    │    │
│   │  │        │                      ▼                              │    │    │
│   │  │        │           ┌─────────────────────┐                   │    │    │
│   │  │        │           │    task_123.mcap    │                   │    │    │
│   │  │        │           │  ┌───────────────┐  │                   │    │    │
│   │  │        │           │  │ MCAP Header   │  │                   │    │    │
│   │  │        │           │  │ - profile     │  │                   │    │    │
│   │  │        │           │  │ - library     │  │                   │    │    │
│   │  │        │           │  ├───────────────┤  │                   │    │    │
│   │  │        │           │  │ Metadata[0]   │◄─┼── Task Context    │    │    │
│   │  │        │           │  │ "axon.task"   │  │                   │    │    │
│   │  │        │           │  ├───────────────┤  │                   │    │    │
│   │  │        │           │  │ Metadata[1]   │◄─┼── Device Info     │    │    │
│   │  │        │           │  │ "axon.device" │  │                   │    │    │
│   │  │        │           │  ├───────────────┤  │                   │    │    │
│   │  │        │           │  │ Metadata[2]   │◄─┼── Recording Info  │    │    │
│   │  │        │           │  │ "axon.rec"    │  │                   │    │    │
│   │  │        │           │  ├───────────────┤  │                   │    │    │
│   │  │        │           │  │ Schema[...]   │  │                   │    │    │
│   │  │        │           │  │ Channel[...]  │  │                   │    │    │
│   │  │        │           │  │ Message[...]  │  │                   │    │    │
│   │  │        │           │  │ ...           │  │                   │    │    │
│   │  │        │           │  └───────────────┘  │                   │    │    │
│   │  │        │           └─────────────────────┘                   │    │    │
│   │  │        │                                                     │    │    │
│   │  │        └───────────► Sidecar JSON Generator                  │    │    │
│   │  │                              │                               │    │    │
│   │  │                              ▼                               │    │    │
│   │  │                    ┌─────────────────────┐                   │    │    │
│   │  │                    │   task_123.json     │                   │    │    │
│   │  │                    │  (quick-parse meta) │                   │    │    │
│   │  │                    └─────────────────────┘                   │    │    │
│   │  └──────────────────────────────────────────────────────────────┘    │    │
│   └─────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
│                              │                                                │
│                              ▼                                                │
│                                                                               │
│   ┌─────────────────────────────────────────────────────────────────────┐    │
│   │                     Edge Storage (Synapse)                           │    │
│   │                                                                      │    │
│   │   /recordings/                                                       │    │
│   │   ├── task_123.mcap     ◄── Self-describing, all metadata embedded   │    │
│   │   ├── task_123.json     ◄── Fast metadata parsing                    │    │
│   │   ├── task_124.mcap                                                  │    │
│   │   ├── task_124.json                                                  │    │
│   │   └── ...                                                            │    │
│   │                                                                      │    │
│   │   Ray/Daft can:                                                      │    │
│   │   1. Quick-scan .json files for task filtering                       │    │
│   │   2. Open .mcap and read embedded metadata if .json missing          │    │
│   │   3. Process data knowing full task context                          │    │
│   └─────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
│                              │                                                │
│                              ▼                                                │
│                                                                               │
│   ┌─────────────────────────────────────────────────────────────────────┐    │
│   │                        Cloud Storage                                 │    │
│   │                                                                      │    │
│   │   Files remain self-describing regardless of:                        │    │
│   │   - Path changes                                                     │    │
│   │   - Bucket reorganization                                            │    │
│   │   - File renaming                                                    │    │
│   └─────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Metadata Schema

### MCAP Metadata Records

MCAP supports multiple `Metadata` records, each with a `name` and key-value `metadata` map. We define five standard metadata records:

#### 1. Sidecar Envelope Metadata (`axon.sidecar`)

Top-level fields needed to reconstruct the JSON sidecar envelope.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `version` | string | ✓ | Sidecar schema version | `"1.0"` |
| `mcap_file` | string | ✓ | Associated MCAP basename | `"task_20251220_143052_abc123.mcap"` |

#### 2. Task Context Metadata (`axon.task`)

Primary task identification and context from the data collector.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `task_id` | string | ✓ | Server-assigned unique task identifier | `"task_20251220_143052_abc123"` |
| `order_id` | string | | Parent order/job identifier | `"order_batch_2025Q4_001"` |
| `scene` | string | ✓ | Primary scene/context label | `"warehouse_picking"` |
| `subscene` | string | | Sub-scene/sub-context label | `"shelf_approach"` |
| `skills` | JSON string array | | Skill identifiers | `["grasp","place","navigate"]` |
| `factory` | string | ✓ | Factory identifier (which factory produced this data) | `"factory_shanghai_01"` |
| `data_collector_id` | string | | Data collector instance ID | `"collector_east_01"` |
| `operator_name` | string | | Human operator identifier | `"john.doe"` |

#### 3. Device Metadata (`axon.device`)

Robot and hardware identification.

| Key | Type | Required | Source | Description | Example |
|-----|------|----------|--------|-------------|---------|
| `device_id` | string | ✓ | Service | Unique robot/device identifier | `"robot_arm_001"` |
| `device_model` | string | | Config | Robot model/type | `"franka_panda"` |
| `device_serial` | string | | Config | Hardware serial number | `"FP-2024-00123"` |
| `hostname` | string | | Runtime | Device hostname (auto-detected) | `"robot-arm-001.local"` |
| `ros_distro` | string | | Runtime | ROS distribution (auto-detected) | `"humble"` |

**Device Metadata Configuration Resolution Order**:

For `device_model` and `device_serial`, the recorder resolves values in this order (first match wins):

1. **Environment variables** (highest priority - per-deployment overrides)
   ```bash
   export AXON_DEVICE_MODEL="franka_panda"
   export AXON_DEVICE_SERIAL="FP-2024-00123"
   ```

2. **YAML config file** (`default_config.yaml`)
   ```yaml
   device:
     model: "franka_panda"
     serial: "FP-2024-00123"
   ```

3. **ROS parameter server** (lowest priority - runtime configuration)
   - ROS 1: `~device_model`, `~device_serial`
   - ROS 2: `device_model`, `device_serial` parameters

**Auto-detected fields**:
- `hostname`: Obtained via `gethostname()` system call
- `ros_distro`: Obtained from `$ROS_DISTRO` environment variable

#### 4. Recording Metadata (`axon.recording`)

Recording session and software information.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `recorder_version` | string | ✓ | axon_recorder version | `"1.2.0"` |
| `recording_started_at` | string | ✓ | ISO 8601 timestamp | `"2025-12-20T14:30:52.123Z"` |
| `recording_finished_at` | string | ✓ | ISO 8601 timestamp | `"2025-12-20T15:45:30.456Z"` |
| `duration_sec` | string | ✓ | Recording duration in seconds | `"4478.333"` |
| `message_count` | string | ✓ | Total messages recorded | `"1523456"` |
| `file_size_bytes` | string | ✓ | Final file size, zero-padded to 20 digits in MCAP metadata | `"0000000002147483648"` |
| `topics_recorded` | JSON string array | | Topic list sorted lexicographically by topic | `["/camera/image","/joint_states"]` |

`checksum_sha256` is intentionally excluded from MCAP metadata because the SHA-256 digest is computed after the MCAP file is finalized. The checksum is written only to the JSON sidecar or upload/Keystone state.

#### 5. Topic Summary Metadata (`axon.topics`)

Per-topic statistics needed to reconstruct `topics_summary` in the sidecar JSON.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `topics_summary` | JSON string array | ✓ | Topic summary entries sorted lexicographically by `topic` | `[{"topic":"/camera/image","message_type":"sensor_msgs/Image","message_count":300,"frequency_hz":30.0}]` |

Each `topics_summary` entry contains:

| Key | Type | Required | Description |
|-----|------|----------|-------------|
| `topic` | string | ✓ | Topic name |
| `message_type` | string | ✓ | Middleware message type |
| `message_count` | integer | ✓ | Messages recorded for the topic |
| `frequency_hz` | number | ✓ | Average observed frequency |

### Metadata Field Encoding Rules

All MCAP metadata values are strings. Scalar string fields are stored directly after sanitization. Numeric fields are stored as decimal strings and parsed back to JSON numbers by downstream readers. Structured fields (`skills`, `topics_recorded`, and `topics_summary`) are compact JSON strings so the sidecar JSON can be reconstructed without delimiter ambiguity. The final `file_size_bytes` value is reserved as a fixed-width, zero-padded decimal string before close and patched in place after close writes the MCAP footer and summary.

### MCAP Header Fields

In addition to Metadata records, we set standard MCAP Header fields:

| Field | Value | Description |
|-------|-------|-------------|
| `profile` | `"ros1"` or `"ros2"` | ROS message encoding profile |
| `library` | `"axon_recorder/{version}"` | Recording library identifier |

## Sidecar JSON Specification

### File Naming Convention

For each MCAP file, generate a corresponding JSON sidecar:

```
task_20251220_143052_abc123.mcap  →  task_20251220_143052_abc123.json
```

### JSON Schema

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "required": ["version", "mcap_file", "task", "device", "recording", "topics_summary"],
  "properties": {
    "version": {
      "type": "string",
      "description": "Sidecar schema version",
      "const": "1.0"
    },
    "mcap_file": {
      "type": "string",
      "description": "Associated MCAP filename (basename only)"
    },
    "task": {
      "type": "object",
      "description": "Task context metadata (mirrors axon.task)",
      "required": ["task_id", "scene", "factory"],
      "properties": {
        "task_id": { "type": "string" },
        "order_id": { "type": "string" },
        "scene": { "type": "string" },
        "subscene": { "type": "string" },
        "skills": { "type": "array", "items": { "type": "string" } },
        "factory": { "type": "string" },
        "data_collector_id": { "type": "string" },
        "operator_name": { "type": "string" }
      }
    },
    "device": {
      "type": "object",
      "description": "Device metadata (mirrors axon.device)",
      "required": ["device_id"],
      "properties": {
        "device_id": { "type": "string" },
        "device_model": { "type": "string" },
        "device_serial": { "type": "string" },
        "hostname": { "type": "string" },
        "ros_distro": { "type": "string" }
      }
    },
    "recording": {
      "type": "object",
      "description": "Recording metadata (mirrors axon.recording)",
      "required": ["recorder_version", "recording_started_at", "recording_finished_at", "duration_sec", "message_count", "file_size_bytes"],
      "properties": {
        "recorder_version": { "type": "string" },
        "recording_started_at": { "type": "string", "format": "date-time" },
        "recording_finished_at": { "type": "string", "format": "date-time" },
        "duration_sec": { "type": "number" },
        "message_count": { "type": "integer" },
        "file_size_bytes": { "type": "integer" },
        "checksum_sha256": { "type": "string" },
        "topics_recorded": { "type": "array", "items": { "type": "string" } }
      }
    },
    "topics_summary": {
      "type": "array",
      "description": "Per-topic statistics for quick filtering",
      "items": {
        "type": "object",
        "properties": {
          "topic": { "type": "string" },
          "message_type": { "type": "string" },
          "message_count": { "type": "integer" },
          "frequency_hz": { "type": "number" }
        }
      }
    }
  }
}
```

### Example Sidecar JSON

```json
{
  "version": "1.0",
  "mcap_file": "task_20251220_143052_abc123.mcap",
  "task": {
    "task_id": "task_20251220_143052_abc123",
    "order_id": "order_batch_2025Q4_001",
    "scene": "warehouse_picking",
    "subscene": "shelf_approach",
    "skills": ["grasp", "place", "navigate"],
    "factory": "factory_shanghai_01",
    "data_collector_id": "collector_east_01",
    "operator_name": "john.doe"
  },
  "device": {
    "device_id": "robot_arm_001",
    "device_model": "franka_panda",
    "device_serial": "FP-2024-00123",
    "hostname": "robot-arm-001.local",
    "ros_distro": "humble"
  },
  "recording": {
    "recorder_version": "1.2.0",
    "recording_started_at": "2025-12-20T14:30:52.123Z",
    "recording_finished_at": "2025-12-20T15:45:30.456Z",
    "duration_sec": 4478.333,
    "message_count": 1523456,
    "file_size_bytes": 2147483648,
    "checksum_sha256": "a1b2c3d4e5f6...",
    "topics_recorded": [
      "/camera/image_raw",
      "/joint_states",
      "/tf",
      "/robot_state"
    ]
  },
  "topics_summary": [
    {
      "topic": "/camera/image_raw",
      "message_type": "sensor_msgs/Image",
      "message_count": 134478,
      "frequency_hz": 30.0
    },
    {
      "topic": "/joint_states",
      "message_type": "sensor_msgs/JointState",
      "message_count": 447833,
      "frequency_hz": 100.0
    },
    {
      "topic": "/tf",
      "message_type": "tf2_msgs/TFMessage",
      "message_count": 895666,
      "frequency_hz": 200.0
    },
    {
      "topic": "/robot_state",
      "message_type": "custom_msgs/RobotState",
      "message_count": 45479,
      "frequency_hz": 10.0
    }
  ]
}
```

## API Changes

### Extended CachedRecordingConfig.srv

Add new fields to support additional metadata:

```diff
# Request - Cache configuration for upcoming recording
# ============================================================================

# Task & Device Identification
string task_id                # Server-assigned task identifier (required for server mode)
string device_id              # Robot/device identifier
string data_collector_id      # Data collector identifier
+string order_id              # Parent order/job identifier (optional)
+string operator_name         # Human operator name (optional)

# Recording Context
string scene                  # Recording scene/context label
string subscene               # Recording subscene/sub-context label
string[] skills               # Associated skills for this recording
string factory                # Factory identifier (which factory produced this data)

# Topic Selection
string[] topics               # Topics to record (empty = use default from YAML config)

# Server Callback Integration (leave empty for ROS-only mode)
string start_callback_url     # POST notification when recording starts
string finish_callback_url    # POST notification when recording finishes
string user_token             # JWT token for callback authentication

---
# Response
# ============================================================================
bool success                  # True if configuration cached successfully
string message                # Human-readable status message
```

**Default behaviors**:
- MCAP metadata injection is always attempted when a task config is present.
- Sidecar JSON generation is enabled by default for edge processing, but can be disabled with `metadata.sidecar.enabled: false` or `recording.sidecar.enabled: false`.
- SHA-256 checksum is computed when the sidecar is generated. If sidecar generation is disabled, the finish callback and status report `sidecar_path: null` and `sidecar_generated: false`.

This keeps embedded MCAP metadata independent from the optional sidecar path: disabling the sidecar must not prevent a recording from finalizing or from carrying `axon.task`, `axon.device`, `axon.recording`, and `axon.topics` records.

### Extended Finish Callback Payload

Include metadata summary in finish callback:

```json
{
  "task_id": "task_20251220_143052_abc123",
  "device_id": "robot_arm_001",
  "status": "finished",
  "started_at": "2025-12-20T14:30:52.123Z",
  "finished_at": "2025-12-20T15:45:30.456Z",
  "duration_sec": 4478.333,
  "message_count": 1523456,
  "file_size_bytes": 2147483648,
  "output_path": "/data/recordings/task_20251220_143052_abc123.mcap",
  "sidecar_path": "/data/recordings/task_20251220_143052_abc123.json",
  "sidecar_enabled": true,
  "sidecar_generated": true,
  "topics": ["/camera/image_raw", "/joint_states", "/tf"],
  "metadata": {
    "scene": "warehouse_picking",
    "subscene": "shelf_approach",
    "skills": ["grasp", "place", "navigate"],
    "factory": "factory_shanghai_01"
  },
  "error": null
}
```

When sidecar generation is disabled or fails after the MCAP is finalized:

```json
{
  "sidecar_path": null,
  "sidecar_enabled": false,
  "sidecar_generated": false
}
```

Downstream clients must treat `sidecar_path: null` as a clear absence of a JSON sidecar, not as an empty path to open.

### Keystone Time-Gap Diagnostics

When the recorder runs in WebSocket client mode, inbound Keystone messages may include a millisecond epoch timestamp in one of these fields: `timestamp_ms`, `server_time_ms`, `keystone_time_ms`, or `timestamp`. The recorder compares that value to local wall-clock receive time and exposes the latest check in `/rpc/status` under `keystone_time_gap`:

```json
{
  "enabled": true,
  "status": "normal",
  "reliable": true,
  "offset_ms": 12,
  "absolute_offset_ms": 12,
  "warning_threshold_ms": 1000,
  "critical_threshold_ms": 5000,
  "stale_after_ms": 60000,
  "checked_at_ms": 1766200000000,
  "reason": "ok"
}
```

Status values:

| Status | Meaning |
|--------|---------|
| `normal` | Offset is below the warning threshold |
| `warning` | Absolute offset is at or above `time_gap_warning_threshold_ms` |
| `critical` | Absolute offset is at or above `time_gap_critical_threshold_ms` |
| `unreliable` | Keystone timestamp is missing, invalid, disabled, or stale |
| `unavailable` | Recorder is not in WebSocket client mode or the client is not running |

Configuration:

```yaml
rpc:
  mode: ws_client
  ws_client:
    url: ws://keystone:8090/rpc
    time_gap_check_enabled: true
    time_gap_warning_threshold_ms: 1000
    time_gap_critical_threshold_ms: 5000
    time_gap_stale_after_ms: 60000
```

The recorder logs status transitions for this check without logging authentication tokens or sensitive request fields.

### Incident Debug Bundle

An optional incident debug bundle can be emitted after MCAP close. It is disabled by default because it copies the finalized MCAP. A bundle failure is non-fatal and must not modify or delete the original MCAP.

Configuration:

```yaml
metadata:
  incident_bundle:
    enabled: true
    directory: /tmp/axon-incident-bundles
```

Bundle contents:

| File | Description |
|------|-------------|
| `recording.mcap` | Copy of the finalized MCAP |
| `sidecar.json` | Copy of the JSON sidecar when generated |
| `manifest.json` | Redacted manifest with artifact paths, task/config summary, recorder version, git SHA, runtime context, and diagnostic snapshot |

The manifest recursively redacts sensitive keys including `token`, `secret`, `password`, `credential`, `authorization`, `access_key`, and callback URL fields. The same redaction is used for bundle manifests and WebSocket invalid-message logs.

## Implementation Details

### MCAP Metadata Writing

MCAP metadata records should be written at file finalization to include accurate statistics:

```cpp
// Pseudo-code for metadata injection
void McapWriter::finalize() {
    // 1. Write sidecar envelope metadata
    mcap::Metadata sidecar_meta;
    sidecar_meta.name = "axon.sidecar";
    sidecar_meta.metadata = {
        {"version", "1.0"},
        {"mcap_file", getBasename(output_path_)}
    };
    writer_.write(sidecar_meta);

    // 2. Write task context metadata
    mcap::Metadata task_meta;
    task_meta.name = "axon.task";
    task_meta.metadata = {
        {"task_id", config_.task_id},
        {"scene", config_.scene},
        {"subscene", config_.subscene},
        {"skills", jsonArray(config_.skills)},
        {"factory", config_.factory},
        {"operator_name", config_.operator_name},
        {"order_id", config_.order_id},
        {"data_collector_id", config_.data_collector_id}
    };
    writer_.write(task_meta);

    // 3. Write device metadata
    mcap::Metadata device_meta;
    device_meta.name = "axon.device";
    device_meta.metadata = {
        {"device_id", config_.device_id},
        {"device_model", getDeviceModel()},
        {"device_serial", getDeviceSerial()},
        {"hostname", getHostname()},
        {"ros_distro", getRosDistro()}
    };
    writer_.write(device_meta);

    // 4. Write recording metadata. file_size_bytes is patched after close.
    mcap::Metadata recording_meta;
    recording_meta.name = "axon.recording";
    recording_meta.metadata = {
        {"recorder_version", AXON_RECORDER_VERSION},
        {"recording_started_at", formatISO8601(start_time_)},
        {"recording_finished_at", formatISO8601(now())},
        {"duration_sec", std::to_string(getDurationSec())},
        {"message_count", std::to_string(total_message_count_)},
        {"file_size_bytes", "00000000000000000000"},
        {"topics_recorded", jsonArray(sorted(recorded_topics_))}
    };
    writer_.write(recording_meta);

    // 5. Write deterministic topic summary metadata.
    mcap::Metadata topics_meta;
    topics_meta.name = "axon.topics";
    topics_meta.metadata = {
        {"topics_summary", jsonArray(sorted(topic_statistics_))}
    };
    writer_.write(topics_meta);

    // 6. Close MCAP file
    writer_.close();

    // 7. Patch final file size into the fixed-width MCAP metadata value.
    patchFileSizeMetadata(output_path_, getFileSize());

    // 8. Compute checksum (always enabled). The checksum is not written into MCAP.
    std::string checksum = computeSHA256(output_path_);

    // 9. Generate sidecar JSON (always enabled)
    generateSidecarJson(checksum);
}
```

### Sidecar JSON Generation

```cpp
void McapWriter::generateSidecarJson(const std::string& checksum) {
    nlohmann::json sidecar;

    sidecar["version"] = "1.0";
    sidecar["mcap_file"] = getBasename(output_path_);

    // Task metadata
    sidecar["task"] = {
        {"task_id", config_.task_id},
        {"order_id", config_.order_id},
        {"scene", config_.scene},
        {"subscene", config_.subscene},
        {"skills", config_.skills},
        {"factory", config_.factory},
        {"data_collector_id", config_.data_collector_id},
        {"operator_name", config_.operator_name}
    };

    // Device metadata
    sidecar["device"] = {
        {"device_id", config_.device_id},
        {"device_model", getDeviceModel()},
        {"device_serial", getDeviceSerial()},
        {"hostname", getHostname()},
        {"ros_distro", getRosDistro()}
    };

    // Recording metadata. checksum_sha256 exists only in the sidecar.
    sidecar["recording"] = {
        {"recorder_version", AXON_RECORDER_VERSION},
        {"recording_started_at", formatISO8601(start_time_)},
        {"recording_finished_at", formatISO8601(finish_time_)},
        {"duration_sec", getDurationSec()},
        {"message_count", total_message_count_},
        {"file_size_bytes", getFileSize()},
        {"checksum_sha256", checksum},
        {"topics_recorded", sorted(recorded_topics_)}
    };

    // Topics summary
    sidecar["topics_summary"] = nlohmann::json::array();
    for (const auto& [topic, stats] : sorted(topic_statistics_)) {
        sidecar["topics_summary"].push_back({
            {"topic", topic},
            {"message_type", stats.message_type},
            {"message_count", stats.message_count},
            {"frequency_hz", stats.computeFrequency()}
        });
    }

    // Atomic write: write to temp file, then rename
    std::string json_path = replaceExtension(output_path_, ".json");
    std::string tmp_path = json_path + ".tmp";
    std::ofstream ofs(tmp_path);
    ofs << sidecar.dump(2);
    ofs.close();
    std::filesystem::rename(tmp_path, json_path);
}
```

### Reading Metadata (Edge/Cloud Processing)

Python example for reading metadata with Ray/Daft:

```python
import json
from pathlib import Path
from mcap.reader import make_reader

def read_metadata_fast(mcap_path: str) -> dict:
    """Fast metadata read from sidecar JSON."""
    json_path = Path(mcap_path).with_suffix('.json')
    if json_path.exists():
        with open(json_path) as f:
            return json.load(f)
    # Fallback to MCAP metadata
    return read_metadata_from_mcap(mcap_path)

def read_metadata_from_mcap(mcap_path: str) -> dict:
    """Read metadata directly from MCAP file."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)

        metadata_records = {}
        for name, metadata in reader.iter_metadata():
            metadata_records[name] = dict(metadata)

        recording = metadata_records["axon.recording"]
        return {
            "version": metadata_records["axon.sidecar"]["version"],
            "mcap_file": metadata_records["axon.sidecar"]["mcap_file"],
            "task": {
                **{k: v for k, v in metadata_records["axon.task"].items() if k != "skills"},
                "skills": json.loads(metadata_records["axon.task"].get("skills", "[]")),
            },
            "device": metadata_records["axon.device"],
            "recording": {
                "recorder_version": recording["recorder_version"],
                "recording_started_at": recording["recording_started_at"],
                "recording_finished_at": recording["recording_finished_at"],
                "duration_sec": float(recording["duration_sec"]),
                "message_count": int(recording["message_count"]),
                "file_size_bytes": int(recording["file_size_bytes"]),
                "topics_recorded": json.loads(recording.get("topics_recorded", "[]")),
            },
            "topics_summary": json.loads(metadata_records["axon.topics"]["topics_summary"]),
        }

# Usage with Daft for batch processing
import daft

@daft.udf(return_dtype=daft.DataType.struct({
    "task_id": daft.DataType.string(),
    "scene": daft.DataType.string(),
    "skills": daft.DataType.list(daft.DataType.string()),
}))
def extract_task_metadata(paths: daft.Series) -> list:
    return [read_metadata_fast(p)["task"] for p in paths.to_pylist()]

# Filter recordings by scene
df = daft.from_glob("/recordings/*.mcap")
df = df.with_column("metadata", extract_task_metadata(df["path"]))
df = df.filter(df["metadata"]["scene"] == "warehouse_picking")
```

## Output Contract

This section defines the contract between `axon_recorder` and any downstream system (e.g., edge storage, cloud pipeline). Downstream systems are treated as a blackbox - they can implement their own ingestion logic based on this contract.

### Recorder Output

For each recording session, `axon_recorder` produces:

| File | Format | Description |
|------|--------|-------------|
| `{task_id}.mcap` | MCAP v1 | Self-describing recording with embedded metadata |
| `{task_id}.json` | JSON | Sidecar file for fast metadata parsing (enabled by default, optional) |

### Output Directory

Files are written to a configurable output directory:

```
{output_directory}/
├── task_20251220_143052_abc123.mcap
├── task_20251220_143052_abc123.json
├── task_20251220_150000_def456.mcap
├── task_20251220_150000_def456.json
└── ...
```

The output directory is configured via:
- YAML config: `output.directory`
- Or ROS parameter: `~output_directory`

### File Guarantees

| Guarantee | Description |
|-----------|-------------|
| **Atomic write** | Files are written to a temp location first, then renamed to final path |
| **Paired output** | `.json` sidecar is written after `.mcap` is finalized when sidecar generation is enabled |
| **Self-describing** | MCAP file contains all metadata even without the sidecar |
| **Consistent naming** | Both files share the same basename (task_id) |
| **Checksum integrity** | SHA-256 checksum is included in sidecar when sidecar generation is enabled |

### Downstream System Responsibilities

Any downstream system (Synapse, cloud uploader, etc.) should:

1. **Watch** the output directory for new files
2. **Parse** the `.json` sidecar for fast metadata access when present (or read MCAP metadata as fallback)
3. **Validate** required fields are present before processing
4. **Handle** the files according to its own workflow (upload, archive, etc.)

### Recommended Validation Checklist

Downstream systems may validate:

| Check | Source | Description |
|-------|--------|-------------|
| `task_id` present | JSON/MCAP | Required for traceability |
| `factory` valid | JSON/MCAP | Required for multi-factory deployments |
| `file_size_bytes` matches | JSON vs filesystem | Data integrity check |
| `mcap_file` matches filename | JSON | Sidecar/MCAP pairing check |
| `checksum_sha256` valid | JSON vs computed | Integrity verification (always available) |
| `message_count > 0` | JSON/MCAP | Non-empty recording check |

## Performance Considerations

### Write Performance

| Operation | Timing | Impact |
|-----------|--------|--------|
| MCAP metadata records | At finalization | ~1ms (negligible) |
| Sidecar JSON generation | At finalization | ~5ms |
| SHA-256 checksum | At finalization | ~10ms/GB (always computed) |

**Note**: All operations are performed at finalization. SHA-256 checksum adds ~10ms/GB overhead but ensures data integrity for all recordings. For a typical 2GB recording, this adds ~20ms to finalization time.

### Read Performance

| Method | Time (typical) | Use Case |
|--------|---------------|----------|
| JSON sidecar parse | ~1ms | Batch filtering, indexing |
| MCAP metadata read | ~50ms | Fallback, verification |
| Full MCAP scan | seconds-minutes | Data processing |

**Recommendation**: Always use JSON sidecar for metadata queries when available.

## Security Considerations

1. **Metadata Sanitization**: All string values should be sanitized before writing
2. **Size Limits**: Enforce maximum lengths on metadata fields to prevent abuse
3. **No Secrets**: Never store credentials, tokens, or PII in metadata
4. **Checksum Integrity**: SHA-256 checksum (always computed) enables tamper detection

### Field Length Limits

| Field | Max Length | Rationale |
|-------|------------|-----------|
| task_id | 256 | UUID + timestamp + suffix |
| scene/subscene | 128 | Human-readable labels |
| skills (each) | 64 | Skill identifiers |
| factory | 128 | Factory identifier |
| operator_name | 128 | Human name |
| *_id fields | 256 | UUIDs or compound IDs |

## Testing Strategy

### Unit Tests

1. **Metadata writing**: Verify MCAP metadata records are correctly formatted
2. **JSON generation**: Verify sidecar JSON matches schema
3. **Field sanitization**: Verify special characters are handled
4. **Size limits**: Verify oversized fields are truncated

### Integration Tests

1. **End-to-end**: Record → Finalize → Verify MCAP metadata → Verify JSON sidecar
2. **Edge compatibility**: JSON can be parsed by Synapse validators
3. **Python SDK**: Metadata can be read with mcap Python package

### Example Test

```cpp
TEST(MetadataInjectionTest, WriteAndReadTaskMetadata) {
    // Setup
    RecordingConfig config;
    config.task_id = "test_task_001";
    config.scene = "test_scene";
    config.skills = {"skill_a", "skill_b"};

    // Record
    McapWriter writer(config);
    writer.open("/tmp/test.mcap");
    writer.writeMessage(...);
    writer.finalize();

    // Verify MCAP metadata
    mcap::McapReader reader;
    reader.open("/tmp/test.mcap");
    auto metadata = reader.metadata();

    ASSERT_TRUE(metadata.contains("axon.task"));
    EXPECT_EQ(metadata["axon.task"]["task_id"], "test_task_001");
    EXPECT_EQ(metadata["axon.task"]["scene"], "test_scene");
    EXPECT_EQ(metadata["axon.task"]["skills"], "skill_a,skill_b");

    // Verify JSON sidecar
    std::ifstream ifs("/tmp/test.json");
    auto json = nlohmann::json::parse(ifs);

    EXPECT_EQ(json["version"], "1.0");
    EXPECT_EQ(json["task"]["task_id"], "test_task_001");
    EXPECT_EQ(json["task"]["scene"], "test_scene");
    EXPECT_EQ(json["task"]["skills"], std::vector<std::string>{"skill_a", "skill_b"});
}
```

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-20 | Initial design |
| 1.1 | 2025-12-21 | Final design: removed `session_id`, `episode_number`, and `tags` fields; renamed `organization` to `factory`; sidecar JSON and SHA-256 checksum are always-on (not configurable) |

## Appendix: MCAP Metadata Record Format

Reference: [MCAP Specification - Metadata Record](https://mcap.dev/spec#metadata-record)

```
Metadata record (opcode 0x0C):
├── name: string        # Metadata name (e.g., "axon.task")
└── metadata: map       # Key-value pairs (string → string)
```

All values in MCAP metadata are strings. For structured data (arrays, numbers), serialize to string:
- Arrays: comma-separated values (e.g., `"skill_a,skill_b"`)
- Numbers: string representation (e.g., `"42"`, `"3.14"`)
- Booleans: `"true"` or `"false"`

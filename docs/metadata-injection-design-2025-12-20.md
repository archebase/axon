# Metadata Injection Capability Design

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

MCAP supports multiple `Metadata` records, each with a `name` and key-value `metadata` map. We define three standard metadata records:

#### 1. Task Context Metadata (`axon.task`)

Primary task identification and context from the data collector.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `task_id` | string | ✓ | Server-assigned unique task identifier | `"task_20251220_143052_abc123"` |
| `order_id` | string | | Parent order/job identifier | `"order_batch_2025Q4_001"` |
| `scene` | string | ✓ | Primary scene/context label | `"warehouse_picking"` |
| `subscene` | string | | Sub-scene/sub-context label | `"shelf_approach"` |
| `skills` | string | | Comma-separated skill identifiers | `"grasp,place,navigate"` |
| `factory` | string | ✓ | Factory identifier (which factory produced this data) | `"factory_shanghai_01"` |
| `data_collector_id` | string | | Data collector instance ID | `"collector_east_01"` |
| `operator_name` | string | | Human operator identifier | `"john.doe"` |

#### 2. Device Metadata (`axon.device`)

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

#### 3. Recording Metadata (`axon.recording`)

Recording session and software information.

| Key | Type | Required | Description | Example |
|-----|------|----------|-------------|---------|
| `recorder_version` | string | ✓ | axon_recorder version | `"1.2.0"` |
| `recording_started_at` | string | ✓ | ISO 8601 timestamp | `"2025-12-20T14:30:52.123Z"` |
| `recording_finished_at` | string | ✓ | ISO 8601 timestamp | `"2025-12-20T15:45:30.456Z"` |
| `duration_sec` | string | ✓ | Recording duration in seconds | `"4478.333"` |
| `message_count` | string | ✓ | Total messages recorded | `"1523456"` |
| `file_size_bytes` | string | ✓ | Final file size | `"2147483648"` |
| `checksum_sha256` | string | ✓ | SHA-256 hash of file content (always computed) | `"a1b2c3..."` |
| `topics_recorded` | string | | Comma-separated topic list | `"/camera/image,/joint_states"` |

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
  "required": ["version", "mcap_file", "task", "device", "recording"],
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

**Always-on behaviors** (not configurable):
- Sidecar JSON generation: always enabled for edge processing
- SHA-256 checksum: always computed for data integrity

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

## Implementation Details

### MCAP Metadata Writing

MCAP metadata records should be written at file finalization to include accurate statistics:

```cpp
// Pseudo-code for metadata injection
void McapWriter::finalize() {
    // 1. Write task context metadata
    mcap::Metadata task_meta;
    task_meta.name = "axon.task";
    task_meta.metadata = {
        {"task_id", config_.task_id},
        {"scene", config_.scene},
        {"subscene", config_.subscene},
        {"skills", join(config_.skills, ",")},
        {"factory", config_.factory},
        {"operator_name", config_.operator_name},
        {"order_id", config_.order_id},
        {"data_collector_id", config_.data_collector_id}
    };
    writer_.write(task_meta);
    
    // 2. Write device metadata
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
    
    // 3. Write recording metadata (with final statistics)
    mcap::Metadata recording_meta;
    recording_meta.name = "axon.recording";
    recording_meta.metadata = {
        {"recorder_version", AXON_RECORDER_VERSION},
        {"recording_started_at", formatISO8601(start_time_)},
        {"recording_finished_at", formatISO8601(now())},
        {"duration_sec", std::to_string(getDurationSec())},
        {"message_count", std::to_string(total_message_count_)},
        {"file_size_bytes", std::to_string(getFileSize())},
        {"topics_recorded", join(recorded_topics_, ",")}
    };
    writer_.write(recording_meta);
    
    // 4. Close MCAP file
    writer_.close();
    
    // 5. Compute checksum (always enabled)
    std::string checksum = computeSHA256(output_path_);
    
    // 6. Generate sidecar JSON (always enabled)
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
    
    // Recording metadata (checksum always included)
    sidecar["recording"] = {
        {"recorder_version", AXON_RECORDER_VERSION},
        {"recording_started_at", formatISO8601(start_time_)},
        {"recording_finished_at", formatISO8601(finish_time_)},
        {"duration_sec", getDurationSec()},
        {"message_count", total_message_count_},
        {"file_size_bytes", getFileSize()},
        {"checksum_sha256", checksum},
        {"topics_recorded", recorded_topics_}
    };
    
    // Topics summary
    sidecar["topics_summary"] = nlohmann::json::array();
    for (const auto& [topic, stats] : topic_statistics_) {
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
        
        metadata = {}
        for name, records in reader.iter_metadata():
            if name == "axon.task":
                metadata["task"] = dict(records)
            elif name == "axon.device":
                metadata["device"] = dict(records)
            elif name == "axon.recording":
                metadata["recording"] = dict(records)
        
        return metadata

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
| `{task_id}.json` | JSON | Sidecar file for fast metadata parsing (always generated) |

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
| **Paired output** | `.json` sidecar is always written after `.mcap` is finalized |
| **Self-describing** | MCAP file contains all metadata even without the sidecar |
| **Consistent naming** | Both files share the same basename (task_id) |
| **Checksum integrity** | SHA-256 checksum is always computed and included in sidecar |

### Downstream System Responsibilities

Any downstream system (Synapse, cloud uploader, etc.) should:

1. **Watch** the output directory for new files
2. **Parse** the `.json` sidecar for fast metadata access (or read MCAP metadata as fallback)
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

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-20 | - | Initial design |
| 1.1 | 2025-12-21 | - | Final design: removed `session_id`, `episode_number`, and `tags` fields; renamed `organization` to `factory`; sidecar JSON and SHA-256 checksum are always-on (not configurable) |

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

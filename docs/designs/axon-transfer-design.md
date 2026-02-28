# axon_transfer Design

**Date:** 2026-02-28
**Status:** Design
**Target Version:** 0.1.0

## Overview

`axon_transfer` is a standalone S3 transfer daemon for the Axon fleet. It runs independently of `axon_recorder`, connects to a fleet management server as a **WebSocket client**, and uploads completed recordings from a local directory to S3-compatible object storage on demand.

The daemon is passive by design: it waits for explicit upload commands from the fleet server and then finds, validates, and uploads the requested recordings using the existing `core/axon_uploader` library. Progress is reported back to the fleet server over the same WebSocket connection.

## Motivation

The original `edge-uploader-design.md` specified the uploader as an in-process module inside `axon_recorder`. Decoupling it into a dedicated daemon brings several advantages:

| Problem (in-process) | Solution (axon_transfer) |
|----------------------|--------------------------|
| Recorder lifecycle tied to upload lifecycle | Uploads continue if recorder restarts |
| No central upload scheduling | Fleet server controls *when* each device uploads |
| Bandwidth spikes interfere with recording | Upload daemon can be throttled independently |
| Harder to deploy separately (e.g., different container) | Separate binary, separate resource limits |
| Fleet-wide batching not possible | Server coordinates uploads across all devices |

## Architecture

```
┌──────────────────────────────────┐
│        Fleet Management Server   │
│                                  │
│  - send upload_request / upload_all
│  - receive status reports        │
└──────────────┬───────────────────┘
               │  WebSocket
               │  ws://{server}/transfer/{device_id}
               ▼
┌──────────────────────────────────────────────────────────────┐
│                   axon_transfer (daemon)                      │
│                                                              │
│  ┌─────────────┐   commands    ┌──────────────────────────┐  │
│  │  WsClient   │──────────────►│   UploadCoordinator      │  │
│  │             │◄──────────────│                          │  │
│  │  reconnect  │   progress    │  dedup via SQLite        │  │
│  │  ping/pong  │   reports     │  task_id → file mapping  │  │
│  └─────────────┘               └────────────┬─────────────┘  │
│                                             │                │
│                                             │ enqueue        │
│                                ┌────────────▼─────────────┐  │
│                                │   FileScanner            │  │
│                                │                          │  │
│                                │  data_dir/               │  │
│                                │  ├── task_abc.mcap       │  │
│                                │  ├── task_abc.json       │  │
│                                │  ├── task_xyz.mcap       │  │
│                                │  └── task_xyz.json       │  │
│                                └────────────┬─────────────┘  │
│                                             │ FileGroup      │
│                                ┌────────────▼─────────────┐  │
│                                │   EdgeUploader           │  │
│                                │  (core/axon_uploader)    │  │
│                                │                          │  │
│                                │  - S3 multipart upload   │  │
│                                │  - SQLite state persist  │  │
│                                │  - Exponential retry     │  │
│                                └────────────┬─────────────┘  │
│                                             │                │
└─────────────────────────────────────────────┼────────────────┘
                                              │
                                              ▼
                                    ┌─────────────────────┐
                                    │  S3 / MinIO storage │
                                    └─────────────────────┘
```

**Relationship with axon_recorder:**
- `axon_recorder` writes completed recordings to `data_dir` (MCAP + JSON sidecar)
- `axon_transfer` reads from the same `data_dir`; it never writes to it
- Both can run concurrently; `axon_transfer` only picks up files after the recorder has finalized them (JSON sidecar present = recording is complete)

## WebSocket Client Design

### Connection Endpoint

```
ws://{server_host}:{server_port}/transfer/{device_id}
```

The `device_id` is embedded in the URL path. The fleet server uses this to identify which device is connecting without requiring a separate authentication handshake. No query parameters are needed for the initial connection.

**Example:**
```
ws://fleet.factory-a.local:8090/transfer/robot_arm_01
```

### Connection Lifecycle

```
                      start()
                        │
                        ▼
                  ┌─────────────┐
        ┌────────►│ CONNECTING  │◄────────────────────────┐
        │         └──────┬──────┘                         │
        │                │ TCP connect + WS handshake      │
        │                │ success                         │
        │                ▼                                 │
        │         ┌─────────────┐                         │
        │         │  CONNECTED  │                         │
        │         │             │ send "connected" msg    │
        │         │  read loop  │                         │
        │         │  ping timer │                         │
        │         └──────┬──────┘                         │
        │                │ error / close / ping timeout   │
        │                ▼                                 │
        │         ┌─────────────┐   schedule_reconnect()  │
        └─────────│  WAITING    │─────────────────────────┘
    backoff delay └─────────────┘
    elapsed
```

### Keepalive (Ping/Pong)

The client sends a WebSocket **Ping** frame every `ping_interval_ms` (default 30 s). If a **Pong** frame is not received within `ping_timeout_ms` (default 10 s), the connection is torn down and reconnect is scheduled.

This differs from the recorder's WebSocket server design: here the *client* initiates pings (because the server may not send them).

### Class Interface

```cpp
namespace axon {
namespace transfer {

class WsClient {
public:
  using MessageHandler = std::function<void(const nlohmann::json&)>;
  using ConnectHandler = std::function<void()>;
  using DisconnectHandler = std::function<void()>;

  WsClient(boost::asio::io_context& ioc, const WsConfig& config);

  // Start connect loop (non-blocking; uses ioc).
  void start();

  // Graceful shutdown.
  void stop();

  // Send a JSON message. Thread-safe; queued internally.
  void send(const nlohmann::json& msg);

  void set_message_handler(MessageHandler handler);
  void set_connect_handler(ConnectHandler handler);
  void set_disconnect_handler(DisconnectHandler handler);

  bool is_connected() const;

private:
  void do_resolve();
  void on_resolve(beast::error_code ec, tcp::resolver::results_type results);
  void do_connect(tcp::resolver::results_type results);
  void on_connect(beast::error_code ec, tcp::endpoint endpoint);
  void do_handshake();
  void on_handshake(beast::error_code ec);

  void do_read();
  void on_read(beast::error_code ec, std::size_t bytes);

  void do_write();
  void on_write(beast::error_code ec, std::size_t bytes);

  void on_disconnect(beast::error_code ec);
  void schedule_reconnect();

  void start_ping_timer();
  void on_ping_timer(beast::error_code ec);

  WsConfig config_;
  tcp::resolver resolver_;
  beast::websocket::stream<beast::tcp_stream> ws_;
  net::strand<net::io_context::executor_type> strand_;
  beast::flat_buffer read_buffer_;

  std::queue<std::string> write_queue_;
  std::mutex write_mutex_;
  std::atomic<bool> writing_{false};

  net::steady_timer reconnect_timer_;
  net::steady_timer ping_timer_;
  std::atomic<bool> ping_pending_{false};

  uint32_t reconnect_attempt_{0};
  std::atomic<bool> connected_{false};
  std::atomic<bool> stopped_{false};

  MessageHandler    message_handler_;
  ConnectHandler    connect_handler_;
  DisconnectHandler disconnect_handler_;
};

}  // namespace transfer
}  // namespace axon
```

## Reconnect Strategy

Reconnect is the most critical reliability property of a long-running daemon. `WsClient` applies **exponential backoff with symmetric jitter** to spread reconnect storms across a fleet.

### Algorithm

```
attempt = 0

on_disconnect():
  delay = min(initial_delay_ms × backoff_multiplier^attempt, max_delay_ms)
  jitter = uniform_random(-delay × jitter_factor, +delay × jitter_factor)
  wait(delay + jitter)
  attempt++
  connect()

on_connect_success():
  attempt = 0            ← reset so next disconnect starts from initial_delay
```

### Parameters (defaults)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `initial_delay_ms` | 1 000 ms | Delay before first retry |
| `backoff_multiplier` | 2.0 | Growth factor per attempt |
| `max_delay_ms` | 60 000 ms | Upper bound on delay |
| `jitter_factor` | 0.2 | ±20 % of computed delay |
| Max attempts | ∞ | Daemon never gives up |

### Delay Table (no jitter, illustrative)

| Attempt | Raw delay | With +20% jitter | With -20% jitter |
|---------|-----------|------------------|------------------|
| 1 | 1 s | 1.2 s | 0.8 s |
| 2 | 2 s | 2.4 s | 1.6 s |
| 3 | 4 s | 4.8 s | 3.2 s |
| 4 | 8 s | 9.6 s | 6.4 s |
| 5 | 16 s | 19.2 s | 12.8 s |
| 6 | 32 s | 38.4 s | 25.6 s |
| 7+ | 60 s (cap) | 72 s | 48 s |

### Behaviour During Disconnection

- **Uploads in flight continue unaffected** — `EdgeUploader` runs its own worker threads and SQLite state; it is independent of the WebSocket connection.
- **New upload commands are lost** — the fleet server does not buffer commands. After reconnection the server re-sends or waits for the device's `status` message to decide what to do next.
- **Progress reports are dropped** — there is no outbound buffer during disconnection. The fleet server can issue a `status_query` after reconnection to get current counts.
- On reconnection `axon_transfer` immediately sends a `connected` message (see §Protocol) so the server knows the device is back.

## Message Protocol

All messages are JSON text frames with a mandatory `type` field.

### Server → Transfer (incoming)

#### `upload_request` — Upload a specific recording

```json
{
  "type": "upload_request",
  "task_id": "task_abc123",
  "priority": 1
}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `task_id` | string | yes | Identifies the recording to upload |
| `priority` | int | no | Higher value = higher priority (default: 0) |

Behaviour: the daemon searches `data_dir` for `{task_id}.mcap` + `{task_id}.json`. If the pair is found and not already completed/in-progress in the SQLite state, it is enqueued.

#### `upload_all` — Upload every pending recording

```json
{
  "type": "upload_all"
}
```

Behaviour: the daemon scans `data_dir` for all complete MCAP+JSON pairs and enqueues any that are not already completed in the SQLite state.

#### `cancel` — Cancel a queued upload

```json
{
  "type": "cancel",
  "task_id": "task_abc123"
}
```

Only effective if the upload has not yet started. If the upload is already in progress, the message is acknowledged but no action is taken.

#### `status_query` — Request current status snapshot

```json
{
  "type": "status_query"
}
```

The daemon replies with a `status` message (see below).

---

### Transfer → Server (outgoing)

#### `connected` — Sent immediately after WS handshake succeeds

```json
{
  "type": "connected",
  "timestamp": "2026-02-28T10:00:00.000Z",
  "data": {
    "version": "0.1.0",
    "device_id": "robot_arm_01",
    "pending_count": 3,
    "uploading_count": 0,
    "failed_count": 1
  }
}
```

Allows the fleet server to learn the device's state immediately on reconnect.

#### `upload_started` — Enqueue confirmed, upload worker picked up the job

```json
{
  "type": "upload_started",
  "timestamp": "2026-02-28T10:00:05.000Z",
  "data": {
    "task_id": "task_abc123",
    "files": ["task_abc123.mcap", "task_abc123.json"],
    "total_bytes": 4294967296
  }
}
```

#### `upload_progress` — Periodic progress update (≥ every 5 s while uploading)

```json
{
  "type": "upload_progress",
  "timestamp": "2026-02-28T10:00:10.000Z",
  "data": {
    "task_id": "task_abc123",
    "bytes_uploaded": 1073741824,
    "total_bytes": 4294967296,
    "percent": 25
  }
}
```

#### `upload_complete` — Both MCAP and JSON confirmed in S3

```json
{
  "type": "upload_complete",
  "timestamp": "2026-02-28T10:05:00.000Z",
  "data": {
    "task_id": "task_abc123",
    "bytes_uploaded": 4294967296,
    "duration_ms": 295000
  }
}
```

Per the existing `EdgeUploader` contract: MCAP is uploaded first; JSON is uploaded last. JSON arriving in S3 is the completion signal for downstream systems (Dagster).

#### `upload_failed` — Upload permanently failed (retries exhausted)

```json
{
  "type": "upload_failed",
  "timestamp": "2026-02-28T10:05:00.000Z",
  "data": {
    "task_id": "task_abc123",
    "reason": "S3 connection refused after 5 retries",
    "retry_count": 5
  }
}
```

#### `upload_not_found` — Requested task_id not found in data_dir

```json
{
  "type": "upload_not_found",
  "timestamp": "2026-02-28T10:00:00.100Z",
  "data": {
    "task_id": "task_abc123",
    "detail": "No MCAP file matching task_abc123 in /data/recordings"
  }
}
```

#### `status` — Current upload counts (reply to `status_query`)

```json
{
  "type": "status",
  "timestamp": "2026-02-28T10:00:00.200Z",
  "data": {
    "pending_count": 3,
    "uploading_count": 1,
    "completed_count": 42,
    "failed_count": 1,
    "pending_bytes": 12884901888,
    "bytes_per_sec": 8912896
  }
}
```

### Protocol Error Handling

| Situation | Response |
|-----------|----------|
| Unknown `type` field | Silently ignored; no response sent |
| Missing required field | `upload_not_found` or log warning; no crash |
| Malformed JSON | Logged as warning; connection kept open |

## File Scanner

### File Naming Convention

The scanner assumes that `axon_recorder` writes recordings using the task_id as the filename stem:

```
{data_dir}/
├── {task_id}.mcap     ← recording data
└── {task_id}.json     ← sidecar metadata (presence = recording is finalized)
```

Both files must exist for a recording to be considered complete and uploadable.

### Scan by Task ID

Used for `upload_request`:
1. Construct `{data_dir}/{task_id}.mcap` and `{data_dir}/{task_id}.json`
2. Verify both files exist and are regular files
3. Check `UploadStateManager` — skip if status is `COMPLETED` or `UPLOADING`
4. Return `FileGroup{mcap_path, json_path, task_id, file_size}`

### Bulk Scan

Used for `upload_all`:
1. Iterate `data_dir` recursively for all `*.mcap` files
2. For each MCAP, check for a matching `*.json` sidecar in the same directory
3. Filter out tasks already `COMPLETED` in SQLite
4. Return sorted list of `FileGroup` (oldest mtime first, so backlog drains in order)

### Class Interface

```cpp
namespace axon {
namespace transfer {

struct FileGroup {
  std::filesystem::path mcap_path;
  std::filesystem::path json_path;
  std::string task_id;
  uint64_t mcap_size_bytes;
};

class FileScanner {
public:
  FileScanner(const ScannerConfig& config,
              uploader::UploadStateManager& state_manager);

  // Find a specific recording by task_id.
  // Returns nullopt if not found or already completed.
  std::optional<FileGroup> find(const std::string& task_id) const;

  // Return all complete, non-uploaded recordings in data_dir.
  std::vector<FileGroup> scan_all() const;

private:
  ScannerConfig config_;
  uploader::UploadStateManager& state_manager_;
};

}  // namespace transfer
}  // namespace axon
```

## Upload Coordinator

`UploadCoordinator` is the glue between `WsClient`, `FileScanner`, and `EdgeUploader`. It:

1. Receives upload commands from `WsClient` (via callbacks set by `TransferDaemon`)
2. Calls `FileScanner` to resolve task_ids to `FileGroup`s
3. Enqueues `FileGroup`s into `EdgeUploader`
4. Receives `EdgeUploader` callbacks and forwards progress reports back through `WsClient`

### Deduplication

Before enqueuing, `UploadCoordinator` checks the SQLite state via `UploadStateManager::get()`:
- `COMPLETED` → skip, optionally send `upload_complete` to server for idempotency
- `UPLOADING` → skip, server is already receiving progress events
- `PENDING` → already queued but not started; skip re-enqueue
- Not found → new upload; insert record and enqueue

### Class Interface

```cpp
namespace axon {
namespace transfer {

class UploadCoordinator {
public:
  UploadCoordinator(
    const TransferConfig& config,
    WsClient& ws_client,
    FileScanner& scanner,
    uploader::EdgeUploader& uploader);

  // Handle an upload_request command.
  void on_upload_request(const std::string& task_id, int priority = 0);

  // Handle an upload_all command.
  void on_upload_all();

  // Handle a cancel command.
  void on_cancel(const std::string& task_id);

private:
  void enqueue(const FileGroup& group, int priority);
  void on_upload_started(const std::string& task_id);
  void on_upload_complete(const std::string& task_id, bool success,
                          const std::string& error);

  const TransferConfig& config_;
  WsClient& ws_client_;
  FileScanner& scanner_;
  uploader::EdgeUploader& uploader_;
};

}  // namespace transfer
}  // namespace axon
```

## Component Design (TransferDaemon)

`TransferDaemon` owns all components and manages their lifecycles.

```cpp
namespace axon {
namespace transfer {

class TransferDaemon {
public:
  explicit TransferDaemon(const TransferConfig& config);

  // Start io_context, uploader, and WS client. Blocks until stop() is called.
  void run();

  // Request graceful shutdown (safe to call from signal handler).
  void stop();

private:
  void on_ws_connected();
  void on_ws_disconnected();
  void on_ws_message(const nlohmann::json& msg);

  TransferConfig config_;

  boost::asio::io_context ioc_;
  boost::asio::signal_set signals_;

  uploader::UploadStateManager state_manager_;
  uploader::EdgeUploader       uploader_;
  FileScanner                  scanner_;
  WsClient                     ws_client_;
  UploadCoordinator            coordinator_;
};

}  // namespace transfer
}  // namespace axon
```

## Threading Model

```
┌────────────────────────────────────────────────────────────────┐
│                    io_context thread (main)                     │
│                                                                │
│   WsClient                                                     │
│   ├── TCP resolver / connect                                   │
│   ├── WS handshake                                             │
│   ├── Async read loop        ─── on_message() ──► coordinator  │
│   ├── Write queue (strand)   ◄── send()        ─── coordinator │
│   ├── Ping timer                                               │
│   └── Reconnect timer                                          │
│                                                                │
│   FileScanner::scan_all / find  (called inline on io_context)  │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│               EdgeUploader worker threads (×2)                  │
│                                                                │
│   Worker 0: dequeue → S3 multipart upload → callback          │
│   Worker 1: dequeue → S3 multipart upload → callback          │
│                       │                                        │
│                       └──► UploadCoordinator::on_upload_*()   │
│                            (callback posts to io_context        │
│                             strand to keep WsClient writes      │
│                             thread-safe)                        │
└────────────────────────────────────────────────────────────────┘
```

### Thread Safety Rules

| Component | Thread Safe? | Notes |
|-----------|-------------|-------|
| `WsClient::send()` | Yes | Mutex-guarded write queue |
| `WsClient` read/write | Yes | Single Boost.Asio strand |
| `FileScanner` | Yes (read-only FS) | No mutable state; filesystem reads are safe |
| `UploadCoordinator` callbacks | Yes | Posts to io_context strand before calling `WsClient::send()` |
| `EdgeUploader::enqueue()` | Yes | Documented thread-safe in `edge_uploader.hpp` |
| `UploadStateManager` | Yes | Documented thread-safe (mutex-guarded SQLite) |

**Critical rule:** `EdgeUploader` upload callbacks arrive on worker threads. They must **post** to the `io_context` strand before touching `WsClient`, not call it directly.

```cpp
// Correct pattern inside UploadCoordinator:
uploader_.setCallback([this, &ioc](const std::string& task_id,
                                   bool success,
                                   const std::string& error) {
  net::post(ioc_, [this, task_id, success, error]() {
    on_upload_complete(task_id, success, error);   // now on io_context thread
  });
});
```

## Configuration

### YAML Schema (`transfer_config.yaml`)

```yaml
# Device identification
device_id: "robot_arm_01"          # Required
factory_id: "factory_a"           # Required (used in S3 key: {factory}/{device}/{date}/{task})

# WebSocket client
ws:
  url: "ws://fleet.factory-a.local:8090/transfer"
  ping_interval_ms: 30000          # How often to send WS ping (ms)
  ping_timeout_ms: 10000           # Pong wait timeout before reconnect (ms)
  reconnect:
    initial_delay_ms: 1000         # First retry delay
    backoff_multiplier: 2.0        # Exponential growth factor
    max_delay_ms: 60000            # Upper bound on delay
    jitter_factor: 0.2             # ±20% randomisation

# File scanner
scanner:
  data_dir: "/axon/recording"       # Root directory to scan for recordings
  require_json_sidecar: true       # Skip MCAP files without a matching .json

# S3 uploader (passed to core/axon_uploader)
uploader:
  state_db_path: "/axon/transfer/transfer_state.db"
  delete_after_upload: true        # Set to false to keep local copy after upload
  failed_uploads_dir: "/axon/transfer/failed_uploads/"
  num_workers: 2

  s3:
    endpoint_url: "https://s3.amazonaws.com"
    bucket: "axon-recordings"
    region: "cn-northwest-1"
    access_key: ""                 # Falls back to AWS_ACCESS_KEY_ID env var
    secret_key: ""                 # Falls back to AWS_SECRET_ACCESS_KEY env var
    part_size: 67108864            # 64 MB multipart chunk size
    connect_timeout_ms: 10000
    request_timeout_ms: 300000

  retry:
    max_retries: 5
    initial_delay_ms: 1000
    max_delay_ms: 300000           # 5 minutes
    backoff_base: 2.0
```

### Environment Variable Overrides

| Environment Variable | Config Field |
|----------------------|-------------|
| `AXON_DEVICE_ID` | `device_id` |
| `AXON_FACTORY_ID` | `factory_id` |
| `AXON_TRANSFER_WS_URL` | `ws.url` |
| `AXON_TRANSFER_DATA_DIR` | `scanner.data_dir` |
| `AXON_TRANSFER_DELETE_AFTER_UPLOAD` | `uploader.delete_after_upload` (`true`/`false`) |
| `AXON_S3_ENDPOINT` | `uploader.s3.endpoint_url` |
| `AXON_S3_BUCKET` | `uploader.s3.bucket` |
| `AWS_ACCESS_KEY_ID` | `uploader.s3.access_key` |
| `AWS_SECRET_ACCESS_KEY` | `uploader.s3.secret_key` |

Environment variables take precedence over YAML values.

## Error Handling

| Error Condition | Behaviour |
|-----------------|-----------|
| `upload_request` for unknown task_id | Scan fails → send `upload_not_found`; no crash |
| MCAP present but JSON sidecar missing | Skip file (recording not yet finalized by recorder) |
| File disappears between scan and enqueue | `EdgeUploader` reports failure → send `upload_failed` |
| S3 transient error | `EdgeUploader` retries with backoff (up to `max_retries`) |
| S3 permanent failure | State set to `FAILED`; files moved to `failed_uploads_dir`; send `upload_failed` |
| WS disconnects during upload | Upload continues; progress reports are dropped; server re-queries on reconnect |
| SQLite I/O error | Log fatal + daemon exits (state DB is critical for crash recovery) |
| Disk full on device | `EdgeUploader` emits backpressure warning at 8 GB pending, alert at 20 GB pending |
| Duplicate `upload_request` | Dedup check in coordinator; silently ignored if already UPLOADING/COMPLETED |

## Data Retention Policy

Whether local files are kept after a successful upload is controlled by a single boolean flag:

```yaml
uploader:
  delete_after_upload: true   # default
```

### Default Paths

| Path | Purpose |
|------|---------|
| `/axon/recording/` | Recorded MCAP + JSON files written by `axon_recorder` |
| `/axon/transfer/` | Transfer daemon working directory (SQLite state DB) |
| `/axon/transfer/failed_uploads/` | Permanently failed files moved here for manual inspection |

### Behaviour by Mode

| `delete_after_upload` | After successful upload | After permanent failure |
|-----------------------|------------------------|------------------------|
| `true` (default) | MCAP + JSON deleted from `data_dir` immediately after S3 confirmation | Files moved to `failed_uploads_dir`; originals deleted from `data_dir` |
| `false` | MCAP + JSON kept in `data_dir`; SQLite marks task `COMPLETED` | Files kept in `data_dir`; SQLite marks task `FAILED` |

**Default is `true`** because robot storage is typically limited and the S3 copy is the authoritative record. Set `false` when a local backup is required (e.g., for debugging or secondary processing).

Regardless of this flag, the SQLite state DB (`transfer_state.db`) always records the outcome. The `FileScanner` checks the DB before enqueuing, so tasks marked `COMPLETED` are never re-uploaded even if the original files were not deleted (i.e., `delete_after_upload: false`).

Files that exceed `max_retries` are always moved to `failed_uploads_dir` so they can be inspected or manually retried, independent of the `delete_after_upload` setting.

### Environment Variable Override

```bash
# Keep local copies after upload (useful for debugging)
export AXON_TRANSFER_DELETE_AFTER_UPLOAD=false
```

## Directory Structure

Planned layout after implementation:

```
apps/axon_transfer/
├── CMakeLists.txt
├── config/
│   └── transfer_config.yaml        ← Default/template config
└── src/
    ├── main.cpp                    ← Entry point; signal handling; config load
    ├── transfer_config.hpp         ← Config structs + load_config()
    ├── transfer_config.cpp         ← YAML + env-var parsing
    ├── ws_client.hpp               ← WsClient (Boost.Beast async client)
    ├── ws_client.cpp
    ├── file_scanner.hpp            ← FileScanner + FileGroup
    ├── file_scanner.cpp
    ├── upload_coordinator.hpp      ← UploadCoordinator
    ├── upload_coordinator.cpp
    ├── transfer_daemon.hpp         ← TransferDaemon (top-level orchestrator)
    └── transfer_daemon.cpp
```

## Dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| `core/axon_uploader` | In-repo | S3 multipart upload, SQLite state, retry |
| `core/axon_logging` | In-repo | Structured logging |
| `Boost.Beast` | System | WebSocket client (same as axon_recorder) |
| `Boost.Asio` | System | Async I/O, timers, strands |
| `nlohmann/json` | In-repo (fetched) | JSON message serialisation |
| `yaml-cpp` | System | Config file parsing |
| `OpenSSL` | System | TLS for future WSS support |

## Testing Strategy

### Unit Tests

| Test | Component | What it verifies |
|------|-----------|-----------------|
| `test_ws_client_reconnect` | `WsClient` | Reconnect fires after disconnect; delay increases with each attempt; resets on success |
| `test_ws_client_backoff` | `WsClient` | Delay stays ≤ `max_delay_ms`; jitter is within bounds |
| `test_ws_client_ping_timeout` | `WsClient` | Connection torn down if pong not received within timeout |
| `test_file_scanner_find` | `FileScanner` | Returns `FileGroup` when both files exist; nullopt when JSON missing |
| `test_file_scanner_skip_completed` | `FileScanner` | `scan_all` excludes tasks already COMPLETED in SQLite |
| `test_file_scanner_bulk_order` | `FileScanner` | Oldest files returned first (by mtime) |
| `test_coordinator_dedup` | `UploadCoordinator` | Second `upload_request` for same task_id is ignored when UPLOADING |
| `test_coordinator_not_found` | `UploadCoordinator` | `upload_not_found` sent when task_id is absent from data_dir |

### Integration Tests

1. **WS + mock server**: a simple Boost.Beast WebSocket server in the test; send `upload_request`, verify `upload_started` → `upload_complete` messages.
2. **Reconnect integration**: mock server closes connection; verify client reconnects within expected backoff window; verify `connected` message is re-sent.
3. **Upload with MinIO**: full flow against a local MinIO container; verify file appears in bucket after `upload_complete`.

### E2E Tests (Docker)

Run `axon_recorder` + `axon_transfer` + MinIO + a mock fleet server in the same Docker Compose environment:
1. Record a short session → recorder writes MCAP + JSON to shared volume
2. Fleet mock server sends `upload_all`
3. Verify MCAP + JSON appear in MinIO bucket
4. Disconnect fleet server → verify reconnect → send `status_query` → verify counts

## Future Enhancements

| Enhancement | Notes |
|-------------|-------|
| **Directory watcher** (`inotify`) | Auto-enqueue new recordings as they appear; removes need for explicit `upload_all` in steady-state |
| **WSS (TLS)** | `wss://` support via `boost::asio::ssl::stream`; config flag to enable |
| **JWT authentication** | Pass device JWT in the `Sec-WebSocket-Protocol` header during handshake |
| **Bandwidth throttling** | Configurable byte-rate cap in `EdgeUploader` worker loop |
| **Upload scheduling windows** | Only upload between 02:00–05:00 to avoid peak hours; controlled by config |
| **Outbound report buffering** | During WS disconnect, buffer `upload_complete` events and flush on reconnect |
| **Multi-path data_dir** | Support scanning multiple directories for multi-recorder setups |

## Related Documentation

- [edge-uploader-design.md](edge-uploader-design.md) — `core/axon_uploader` library specification
- [websocket-push-design.md](websocket-push-design.md) — recorder→panel WebSocket server (server-side, contrast with the client-side design here)
- [rpc-api-design.md](rpc-api-design.md) — recorder HTTP RPC API

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 0.1 | 2026-02-28 | Initial design |
| 0.2 | 2026-02-28 | Add Data Retention Policy section; set default paths under `/axon/` |

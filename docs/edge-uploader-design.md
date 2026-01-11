# Edge Uploader Design

**Date:** 2025-12-21

## Overview

This document defines the **Edge Uploader** capability for `axon_recorder`, a device-side C++ component responsible for automatically uploading recorded MCAP files and their sidecar JSON metadata to an S3-compatible edge storage layer (e.g., MinIO, Ceph, AWS S3). The Edge Uploader operates as a tightly integrated module within the recorder or as a lightweight companion daemon, ensuring reliable data transfer from device to edge with data integrity guarantees.

## Problem Statement

After recording completes, MCAP files and sidecar metadata reside on local device storage. These files must be transferred to edge storage for downstream processing by the Edge Manager (Ray + Daft + Dagster). Current challenges:

| Challenge | Impact |
|-----------|--------|
| Network unreliability | Partial uploads, data loss |
| Storage constraints | Device fills up if uploads fail |
| Data integrity | Silent corruption during transfer |
| Security | Credentials exposure, unauthorized access |
| Observability | Unknown upload status, debugging difficulty |

**Solution**: A robust, fault-tolerant Edge Uploader module that monitors the recorder's output directory, uploads files with integrity verification, and provides clear status reporting.

## Design Goals

1. **Reliability First** - Guaranteed delivery with automatic retry and resume
2. **Data Integrity** - SHA-256 verification ensures files are uncorrupted
3. **Resource Efficient** - Minimal CPU/memory footprint on edge device
4. **Security** - Encrypted transport, credential rotation, minimal privileges
5. **Observable** - Clear status reporting for monitoring and debugging
6. **Tight Integration** - Native C++ integration with axon_recorder

## Scope & Non-Goals

### In Scope

| Item | Description |
|------|-------------|
| Device → Edge upload | Upload MCAP + JSON from device to S3-compatible edge storage |
| Structural validation | Header/footer/summary check before upload |
| Crash recovery | Resume incomplete uploads after recorder restart |
| Integrity verification | SHA-256 checksum in metadata |

### Out of Scope (Explicit Non-Goals)

| Item | Rationale |
|------|-----------|
| **MCAP indexing** | MCAP is self-describing; full scans needed for content QA (e.g., black image detection). Sidecar JSON provides metadata. |
| **Schema registry** | Topics can evolve freely; downstream handles new schemas as new data |
| **Multi-file runs** | One task = one MCAP file; no cross-file linking |
| **MCAP → Lance conversion** | May happen at edge, but not in scope for uploader |
| **Extended offline mode** | Network is expected to be available; operator handles outages |
| **Bandwidth throttling** | Out of scope for initial implementation |
| **Multi-tenancy** | Each factory has its own edge platform |
| **Content-level QA** | Black image detection, sensor gaps, etc. happen at edge, not device |

## Performance Requirements & SLOs

### Workload Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| **File size (p90)** | ~4 GB | MCAP files with sensor data |
| **File size (sidecar JSON)** | ~10 KB | Metadata only |
| **Recording frequency** | 1 file per 3-5 minutes | Per device |
| **Files per device per hour** | 12-20 | Peak load assumption |

### Service Level Objectives (SLOs)

| SLO | Target | Measurement |
|-----|--------|-------------|
| **Upload latency (p95)** | < 5 minutes | Time from `finalize()` to S3 confirmation |
| **Upload success rate** | > 99.5% | Successful uploads / total attempts (excluding network outages) |
| **Data loss** | 0% | No recording lost after `finalize()` completes |

### Throughput Calculations

```
Recording rate:    4 GB / 4 min = 1 GB/min = ~17 MB/s
Required network:  17 MB/s sustained (140 Mbps) to keep up with recording
With 2 workers:    Each worker needs ~8.5 MB/s average throughput
```

**Backpressure Trigger**: If pending queue exceeds 2 files (8 GB), emit warning. If exceeds 5 files (20 GB), alert operator.

### Network Outage Handling

When network is unavailable for extended periods:
1. Uploader continues queuing files locally
2. Alerts operator when pending queue exceeds threshold
3. **Operator action required**: Stop data collection and fix network issue
4. On network recovery, uploader drains queue automatically

### Local File Retention

| Scenario | Action |
|----------|--------|
| Upload successful | **Delete immediately** - device copy removed after S3 confirmation |
| Upload failed (retrying) | Keep on device until max retries exhausted |
| Upload permanently failed | Move to `/data/failed_uploads/` for manual review |

### Compliance Requirements

- **GDPR applicable**: Yes
- **Audit logging**: Required - all uploads logged with timestamp, task_id, outcome
- **Data retention**: Edge team owns lifecycle policies on S3 bucket
- **PII handling**: Metadata fields (operator_name) may contain PII - handle per GDPR

## System Architecture

```
┌────────────────────────────────────────────────────────────────────────────────────┐
│                              DEVICE LAYER (C++)                                     │
├────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   ┌──────────────────────────────────────────────────────────────────────────────┐  │
│   │                           axon_recorder Process                               │  │
│   │                                                                               │  │
│   │   ┌───────────────────┐         ┌──────────────────────────────────────┐    │  │
│   │   │  RecordingSession │────────►│         Edge Uploader Module          │    │  │
│   │   │                   │ direct  │                                       │    │  │
│   │   │  • MCAP Writer    │ handoff │  ┌─────────────┐  ┌───────────────┐  │    │  │
│   │   │  • Metadata       │         │  │Upload Queue │  │  S3 Client    │  │    │  │
│   │   │  • Sidecar JSON   │         │  │(lock-free)  │  │  (minio-cpp)  │  │    │  │
│   │   └───────────────────┘         │  └─────────────┘  └───────────────┘  │    │  │
│   │            │                    │         │                │           │    │  │
│   │            ▼                    │         ▼                ▼           │    │  │
│   │   /data/recordings/             │  ┌─────────────┐  ┌───────────────┐  │    │  │
│   │   ├── task_001.mcap             │  │State Manager│  │ Retry Handler │  │    │  │
│   │   ├── task_001.json             │  │  (SQLite)   │  │ (Exp Backoff) │  │    │  │
│   │   └── ...                       │  └─────────────┘  └───────────────┘  │    │  │
│   │                                 │                                       │    │  │
│   │                                 │  Credentials: Environment Variables   │    │  │
│   │                                 │  (AWS_ACCESS_KEY_ID, AWS_SECRET_...)  │    │  │
│   │                                 └──────────────────────────────────────┘    │  │
│   └──────────────────────────────────────────────────────────────────────────────┘  │
│                                          │                                          │
│                                          │ HTTPS (TLS 1.3)                          │
│                                          ▼                                          │
└────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                                           ▼
┌────────────────────────────────────────────────────────────────────────────────────┐
│                          EDGE STORAGE LAYER (S3-Compatible)                         │
├────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   s3://axon-raw-data/                                                         │
│   └── {factory_id}/                                                                 │
│       └── {device_id}/                                                              │
│           └── {date}/                                                               │
│               ├── task_001.mcap                                                     │
│               ├── task_001.json                                                     │
│               ├── task_002.mcap                                                     │
│               ├── task_002.json                                                     │
│               └── ...                                                               │
│                                                                                      │
│   Bucket Policies:                                                                  │
│   • Device can only write to {factory_id}/{device_id}/ prefix                      │
│   • Edge Manager has read access to all prefixes                                   │
│   • Lifecycle: 7 days on edge → upload to cloud → delete from edge                 │
│                                                                                      │
└────────────────────────────────────────────────────────────────────────────────────┘
                                           │
                            ═══════════════╪═══════════════
                                   S3 API Boundary
                            ═══════════════╪═══════════════
                                           │
                                           ▼
┌────────────────────────────────────────────────────────────────────────────────────┐
│                       EDGE COMPUTE LAYER (Python - Separate System)                 │
├────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                      │
│   ┌─────────────────────────────────────────────────────────────────────────────┐  │
│   │                     Dagster + Ray + Daft Pipeline                            │  │
│   │                                                                               │  │
│   │   • Dagster Sensors poll S3 for new .json sidecar files                      │  │
│   │   • Ray distributes MCAP processing across workers                           │  │
│   │   • Daft enables fast metadata queries via sidecar JSON                      │  │
│   │                                                                               │  │
│   │   (See "Downstream Consumer Reference" section for integration details)       │  │
│   └─────────────────────────────────────────────────────────────────────────────┘  │
│                                                                                      │
└────────────────────────────────────────────────────────────────────────────────────┘
```

## Integration Pattern

The Edge Uploader runs as an **in-process module** within the `axon_recorder` process, with direct handoff after recording finalization.

```
┌─────────────────────────────────────────────────────────────┐
│                    axon_recorder Process                     │
│                                                              │
│   RecordingSession::finalize()                              │
│         │                                                    │
│         ├── 1. Write MCAP metadata records                  │
│         ├── 2. Close MCAP file                              │
│         ├── 3. Validate MCAP structure (header + summary)   │
│         ├── 4. Compute SHA-256 checksum                     │
│         ├── 5. Generate sidecar JSON                        │
│         │                                                    │
│         └── 6. uploader_->enqueue(mcap_path, json_path)     │
│                        │                                     │
│                        ▼                                     │
│              ┌─────────────────┐                            │
│              │  Upload Queue   │ ──► Background upload      │
│              │  (lock-free)    │     thread(s)              │
│              └─────────────────┘                            │
└─────────────────────────────────────────────────────────────┘
```

**Why In-Process (No Directory Watching)**:

| Aspect | In-Process Direct Handoff |
|--------|---------------------------|
| Latency | Zero - direct function call |
| Reliability | Guaranteed handoff in `finalize()` |
| Complexity | Simple - no file system events |
| Crash Recovery | State DB resumes pending uploads on restart |
| Resource Usage | Lower - no watcher threads |

**Note**: Directory watching would only be needed for a separate daemon process, which adds complexity without significant benefit for our use case.

### Crash Recovery Flow

When the recorder process restarts after a crash, the uploader automatically resumes incomplete uploads:

```
┌─────────────────────────────────────────────────────────────┐
│                  Recorder Process Restart                    │
│                                                              │
│   RecorderNode::RecorderNode()                              │
│         │                                                    │
│         └── uploader_ = make_unique<EdgeUploader>(config)   │
│                   │                                          │
│                   └── uploader_->start()                    │
│                             │                                │
│                             ▼                                │
│                   ┌─────────────────────────────────┐       │
│                   │  1. Query State DB              │       │
│                   │     SELECT * FROM upload_state  │       │
│                   │     WHERE status IN             │       │
│                   │     ('pending', 'uploading')    │       │
│                   └─────────────────────────────────┘       │
│                             │                                │
│                             ▼                                │
│                   ┌─────────────────────────────────┐       │
│                   │  2. Re-queue incomplete uploads │       │
│                   │     for (auto& record :         │       │
│                   │          state_db->getIncomplete()) │   │
│                   │       queue_->enqueue(record);  │       │
│                   └─────────────────────────────────┘       │
│                             │                                │
│                             ▼                                │
│                   ┌─────────────────────────────────┐       │
│                   │  3. Start worker threads        │       │
│                   │     Workers process queue       │       │
│                   └─────────────────────────────────┘       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**Key Points**:
- State DB persists upload state to disk (SQLite)
- Uploads marked "uploading" at crash time are treated as incomplete
- Files remain on disk until upload is confirmed successful
- No data loss even if crash occurs mid-upload (S3 multipart is atomic)

## Component Design

### 1. Upload Queue

Lock-free queue for pending uploads, optimized for single-producer (recorder) multi-consumer (upload workers) pattern.

**Interface**:

```cpp
// upload_queue.hpp
#pragma once

#include <atomic>
#include <string>
#include <chrono>
#include <optional>

namespace axon {

struct UploadItem {
    std::string mcap_path;          // Local MCAP file path
    std::string json_path;          // Local sidecar JSON path
    std::string s3_key_prefix;      // S3 key prefix (factory/device/date)
    std::string task_id;            // Task identifier
    std::string checksum_sha256;    // Pre-computed checksum from recorder
    uint64_t file_size_bytes;       // File size for progress tracking
    int retry_count = 0;            // Number of retry attempts
    std::chrono::steady_clock::time_point created_at;
    std::chrono::steady_clock::time_point next_retry_at;
};

class UploadQueue {
public:
    explicit UploadQueue(size_t capacity = 1024);
    ~UploadQueue();

    // Producer API (called by recorder)
    bool enqueue(UploadItem item);
    
    // Consumer API (called by upload workers)
    std::optional<UploadItem> dequeue();
    std::optional<UploadItem> dequeue_with_timeout(std::chrono::milliseconds timeout);
    
    // Re-queue for retry (with updated retry_count and next_retry_at)
    bool requeue_for_retry(UploadItem item);
    
    // Status
    size_t size() const;
    bool empty() const;
    
private:
    // Lock-free SPSC queue for main items
    // Priority queue for retry items (protected by mutex)
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace axon
```

**Implementation Notes**:
- Use existing `spsc_queue.hpp` from recorder for lock-free operations
- Separate retry queue with delayed processing
- Priority based on file creation time and retry count

### 2. S3 Client Wrapper

Thin wrapper around **minio-cpp** providing simplified upload API. minio-cpp is designed for S3-compatible storage and works with any provider (AWS S3, MinIO, Ceph, Alibaba OSS, Cloudflare R2, etc.).

**Why minio-cpp over AWS SDK**:

| Aspect | minio-cpp | AWS SDK for C++ |
|--------|-----------|-----------------|
| Size | Lightweight (~2MB) | Heavy (~50MB) |
| Dependencies | libcurl, openssl | Many AWS-specific libs |
| Focus | S3-compatible | AWS-centric |
| Multipart | Automatic | Requires TransferManager |
| Multi-cloud | First-class | Endpoint override |

**Interface**:

```cpp
// s3_client.hpp
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <map>

namespace axon {

struct S3Config {
    std::string endpoint_url;       // e.g., "https://play.min.io" or "https://s3.amazonaws.com"
    std::string bucket;
    std::string region = "us-east-1";
    bool use_ssl = true;
    bool verify_ssl = true;
    
    // Credentials (if not using environment variables)
    std::string access_key;         // Or set AWS_ACCESS_KEY_ID env var
    std::string secret_key;         // Or set AWS_SECRET_ACCESS_KEY env var
    
    // Multipart upload settings (minio-cpp handles this automatically)
    uint64_t part_size = 64 * 1024 * 1024;  // 64MB per part
    
    // Timeouts
    int connect_timeout_ms = 10000;
    int request_timeout_ms = 60000;
};

struct UploadResult {
    bool success;
    std::string etag;               // S3 ETag for verification
    std::string version_id;         // Version ID (if versioning enabled)
    std::string error_message;
    bool is_retryable;              // True for transient errors
};

using ProgressCallback = std::function<void(uint64_t bytes_transferred, uint64_t total_bytes)>;

class S3Client {
public:
    explicit S3Client(const S3Config& config);
    ~S3Client();
    
    // Upload file to S3-compatible storage
    // Automatically uses multipart upload for large files
    UploadResult uploadFile(
        const std::string& local_path,
        const std::string& s3_key,
        const std::map<std::string, std::string>& metadata = {},
        ProgressCallback progress_cb = nullptr
    );
    
    // Check if object exists
    bool objectExists(const std::string& s3_key);
    
    // Get object metadata (for verification)
    std::map<std::string, std::string> getObjectMetadata(const std::string& s3_key);
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace axon
```

**Implementation using minio-cpp**:

```cpp
// s3_client.cpp
#include "s3_client.hpp"

#include <miniocpp/client.h>
#include <fstream>

namespace axon {

struct S3Client::Impl {
    S3Config config;
    std::unique_ptr<minio::s3::Client> client;
    
    void initClient() {
        // Create base URL from endpoint
        minio::s3::BaseUrl base_url(config.endpoint_url);
        base_url.https = config.use_ssl;
        
        // Create credentials provider
        minio::creds::StaticProvider creds_provider(
            config.access_key,
            config.secret_key
        );
        
        // Create client
        client = std::make_unique<minio::s3::Client>(base_url, &creds_provider);
        
        // Configure SSL verification
        if (!config.verify_ssl) {
            client->IgnoreCertCheck();
        }
    }
};

S3Client::S3Client(const S3Config& config) : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
    
    // Load credentials from environment if not provided
    if (impl_->config.access_key.empty()) {
        if (const char* key = std::getenv("AWS_ACCESS_KEY_ID")) {
            impl_->config.access_key = key;
        }
    }
    if (impl_->config.secret_key.empty()) {
        if (const char* key = std::getenv("AWS_SECRET_ACCESS_KEY")) {
            impl_->config.secret_key = key;
        }
    }
    
    impl_->initClient();
}

S3Client::~S3Client() = default;

UploadResult S3Client::uploadFile(
    const std::string& local_path,
    const std::string& s3_key,
    const std::map<std::string, std::string>& metadata,
    ProgressCallback progress_cb
) {
    UploadResult result;
    
    // Prepare upload arguments
    minio::s3::UploadObjectArgs args;
    args.bucket = impl_->config.bucket;
    args.object = s3_key;
    args.filename = local_path;
    
    // Add custom metadata
    for (const auto& [key, value] : metadata) {
        args.user_metadata[key] = value;
    }
    
    // Set progress callback if provided
    if (progress_cb) {
        args.progress_func = [progress_cb](size_t uploaded, size_t total) {
            progress_cb(uploaded, total);
        };
    }
    
    // Perform upload (minio-cpp automatically uses multipart for large files)
    minio::s3::UploadObjectResponse resp = impl_->client->UploadObject(args);
    
    if (resp) {
        result.success = true;
        result.etag = resp.etag;
        result.version_id = resp.version_id;
    } else {
        result.success = false;
        result.error_message = resp.Error().String();
        result.is_retryable = isRetryableError(resp.Error().code);
    }
    
    return result;
}

bool S3Client::objectExists(const std::string& s3_key) {
    minio::s3::StatObjectArgs args;
    args.bucket = impl_->config.bucket;
    args.object = s3_key;
    
    minio::s3::StatObjectResponse resp = impl_->client->StatObject(args);
    return resp;  // Returns true if object exists
}

std::map<std::string, std::string> S3Client::getObjectMetadata(const std::string& s3_key) {
    minio::s3::StatObjectArgs args;
    args.bucket = impl_->config.bucket;
    args.object = s3_key;
    
    minio::s3::StatObjectResponse resp = impl_->client->StatObject(args);
    
    std::map<std::string, std::string> result;
    if (resp) {
        result["etag"] = resp.etag;
        result["size"] = std::to_string(resp.size);
        result["last_modified"] = resp.last_modified.ToString();
        for (const auto& [key, value] : resp.user_metadata) {
            result[key] = value;
        }
    }
    return result;
}

// Helper to classify retryable errors
bool isRetryableError(const std::string& error_code) {
    static const std::set<std::string> retryable = {
        "RequestTimeout",
        "ServiceUnavailable",
        "InternalError",
        "SlowDown",
        "ConnectionReset",
        "ConnectionTimeout",
        "XMinioServerNotInitialized"
    };
    return retryable.count(error_code) > 0;
}

}  // namespace axon
```

### 3. State Manager

Persists upload state for crash recovery using SQLite.

**Interface**:

```cpp
// upload_state_manager.hpp
#pragma once

#include <string>
#include <vector>
#include <optional>
#include <chrono>

namespace axon {

enum class UploadStatus {
    PENDING,
    UPLOADING,
    COMPLETED,
    FAILED
};

struct UploadRecord {
    std::string file_path;
    std::string s3_key;
    std::string task_id;
    std::string factory_id;
    std::string device_id;
    uint64_t file_size_bytes;
    std::string checksum_sha256;
    UploadStatus status;
    int retry_count;
    std::string last_error;
    std::string created_at;     // ISO 8601
    std::string updated_at;     // ISO 8601
    std::string completed_at;   // ISO 8601 (empty if not completed)
};

class UploadStateManager {
public:
    explicit UploadStateManager(const std::string& db_path);
    ~UploadStateManager();
    
    // CRUD operations
    bool insert(const UploadRecord& record);
    bool updateStatus(const std::string& file_path, UploadStatus status, 
                      const std::string& error = "");
    bool markCompleted(const std::string& file_path);
    bool markFailed(const std::string& file_path, const std::string& error);
    bool incrementRetry(const std::string& file_path, const std::string& error);
    bool remove(const std::string& file_path);
    
    // Queries
    std::optional<UploadRecord> get(const std::string& file_path);
    std::vector<UploadRecord> getPending();
    std::vector<UploadRecord> getFailed();
    
    // Crash recovery: get all incomplete uploads (pending + uploading)
    // Called by EdgeUploader::start() on recorder restart
    std::vector<UploadRecord> getIncomplete();
    
    // Statistics
    size_t countByStatus(UploadStatus status);
    uint64_t totalBytesUploaded();
    
    // Cleanup
    int deleteOlderThan(std::chrono::hours age);
    int deleteCompleted();
    
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace axon
```

**SQLite Schema**:

```sql
-- Enable WAL mode for crash safety and concurrent reads
-- This MUST be set before creating tables
PRAGMA journal_mode=WAL;
PRAGMA synchronous=NORMAL;  -- Good balance of safety and performance
PRAGMA busy_timeout=5000;   -- Wait 5s if DB is locked

CREATE TABLE IF NOT EXISTS upload_state (
    file_path TEXT PRIMARY KEY,
    s3_key TEXT NOT NULL,
    task_id TEXT,
    factory_id TEXT,
    device_id TEXT,
    file_size_bytes INTEGER,
    checksum_sha256 TEXT,
    status TEXT NOT NULL CHECK(status IN ('pending', 'uploading', 'completed', 'failed')),
    retry_count INTEGER DEFAULT 0,
    last_error TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL,
    completed_at TEXT
);

CREATE INDEX IF NOT EXISTS idx_status ON upload_state(status);
CREATE INDEX IF NOT EXISTS idx_task_id ON upload_state(task_id);
CREATE INDEX IF NOT EXISTS idx_created_at ON upload_state(created_at);
```

**Why WAL Mode?**

| Mode | Crash Behavior | Performance | Our Choice |
|------|----------------|-------------|------------|
| DELETE (default) | May lose uncommitted | Slower | ❌ |
| WAL | Atomic commits | Faster concurrent | ✅ |

WAL ensures upload state is never corrupted even if recorder crashes mid-write.

### 4. Retry Handler

Implements exponential backoff with jitter for transient failures.

```cpp
// retry_handler.hpp
#pragma once

#include <chrono>
#include <random>

namespace axon {

struct RetryConfig {
    int max_retries = 5;
    std::chrono::milliseconds initial_delay{1000};
    std::chrono::milliseconds max_delay{300000};  // 5 minutes
    double exponential_base = 2.0;
    bool jitter = true;
};

class RetryHandler {
public:
    explicit RetryHandler(const RetryConfig& config = {});
    
    // Calculate delay for next retry
    std::chrono::milliseconds getDelay(int retry_count) const;
    
    // Check if should retry
    bool shouldRetry(int retry_count) const;
    
    // Classify error as retryable
    static bool isRetryableError(const std::string& error_code);
    
private:
    RetryConfig config_;
    mutable std::mt19937 rng_;
};

// Implementation
inline std::chrono::milliseconds RetryHandler::getDelay(int retry_count) const {
    double delay_ms = config_.initial_delay.count() * 
                      std::pow(config_.exponential_base, retry_count);
    delay_ms = std::min(delay_ms, static_cast<double>(config_.max_delay.count()));
    
    if (config_.jitter) {
        std::uniform_real_distribution<> dist(0.5, 1.5);
        delay_ms *= dist(rng_);
    }
    
    return std::chrono::milliseconds(static_cast<int64_t>(delay_ms));
}

inline bool RetryHandler::shouldRetry(int retry_count) const {
    return retry_count < config_.max_retries;
}

inline bool RetryHandler::isRetryableError(const std::string& error_code) {
    // Retryable errors
    static const std::set<std::string> retryable = {
        "RequestTimeout",
        "ServiceUnavailable", 
        "InternalError",
        "SlowDown",
        "RequestTimeTooSkewed",
        "ConnectionReset",
        "ConnectionTimeout"
    };
    return retryable.count(error_code) > 0;
}

}  // namespace axon
```

### 5. Edge Uploader (Main Class)

Orchestrates all components.

```cpp
// edge_uploader.hpp
#pragma once

#include "upload_queue.hpp"
#include "s3_client.hpp"
#include "upload_state_manager.hpp"
#include "retry_handler.hpp"

#include <memory>
#include <thread>
#include <atomic>
#include <vector>

namespace axon {

struct UploaderConfig {
    // S3 configuration
    S3Config s3;
    
    // Retry configuration
    RetryConfig retry;
    
    // Worker configuration
    int num_workers = 2;
    
    // State persistence
    std::string state_db_path = "/var/lib/axon/uploader_state.db";
    
    // Cleanup policy
    bool delete_after_upload = true;       // Delete device copy immediately after successful upload
    std::string failed_uploads_dir = "/data/failed_uploads/";  // Move permanently failed files here
    uint64_t min_free_space_bytes = 20ULL * 1024 * 1024 * 1024;  // 20GB (~5 pending files)
    
    // Backpressure thresholds
    uint64_t warn_pending_bytes = 8ULL * 1024 * 1024 * 1024;   // 8GB (~2 files) - emit warning
    uint64_t alert_pending_bytes = 20ULL * 1024 * 1024 * 1024; // 20GB (~5 files) - alert operator
};

struct UploaderStats {
    std::atomic<uint64_t> files_pending{0};
    std::atomic<uint64_t> files_uploading{0};
    std::atomic<uint64_t> files_completed{0};
    std::atomic<uint64_t> files_failed{0};
    std::atomic<uint64_t> bytes_uploaded{0};
    std::atomic<uint64_t> current_upload_bytes_per_sec{0};
};

class EdgeUploader {
public:
    explicit EdgeUploader(const UploaderConfig& config);
    ~EdgeUploader();
    
    // Lifecycle
    // start() performs crash recovery:
    //   1. Query State DB for uploads with status = 'pending' or 'uploading'
    //   2. Re-queue them for upload
    //   3. Start worker threads
    void start();
    void stop();
    bool isRunning() const;
    
    // Producer API (called by recorder after finalization)
    void enqueue(const std::string& mcap_path, 
                 const std::string& json_path,
                 const std::string& task_id,
                 const std::string& factory_id,
                 const std::string& device_id,
                 const std::string& checksum_sha256);
    
    // Convenience: extract metadata from sidecar JSON
    void enqueueFromSidecar(const std::string& mcap_path, 
                            const std::string& json_path);
    
    // Statistics
    const UploaderStats& stats() const;
    
    // Health check
    struct HealthStatus {
        bool healthy;
        std::string message;
        uint64_t pending_count;
        uint64_t failed_count;
        std::chrono::system_clock::time_point last_successful_upload;
    };
    HealthStatus getHealthStatus() const;
    
private:
    void workerLoop();
    void processItem(const UploadItem& item);
    std::string constructS3Key(const std::string& factory_id,
                                const std::string& device_id,
                                const std::string& task_id,
                                const std::string& extension);
    void cleanupLocalFile(const std::string& path);
    bool checkDiskSpace();
    
    UploaderConfig config_;
    std::unique_ptr<UploadQueue> queue_;
    std::unique_ptr<S3Client> s3_client_;
    std::unique_ptr<UploadStateManager> state_manager_;
    std::unique_ptr<RetryHandler> retry_handler_;
    
    std::vector<std::thread> workers_;
    std::atomic<bool> running_{false};
    
    UploaderStats stats_;
    std::chrono::system_clock::time_point last_successful_upload_;
    mutable std::mutex stats_mutex_;
};

}  // namespace axon
```

## Integration with axon_recorder

### Recording Session Integration

```cpp
// In recording_session.cpp

void RecordingSession::finalize() {
    // ... existing finalization code ...
    
    // 1. Write MCAP metadata records
    writeMetadataRecords();
    
    // 2. Close MCAP file
    mcap_writer_->close();
    
    // 3. Validate MCAP structure before upload
    //    - Validates header magic bytes
    //    - Validates summary section is present and parseable
    //    - Does NOT decode all messages (full content QA happens at edge)
    if (!validateMcapStructure(output_path_)) {
        RCLCPP_ERROR(logger_, "MCAP validation failed, dropping file: %s", 
                     output_path_.c_str());
        std::filesystem::remove(output_path_);
        return;  // Do not upload corrupt files
    }
    
    // 4. Compute SHA-256 checksum
    std::string checksum = computeSHA256(output_path_);
    
    // 5. Generate sidecar JSON
    std::string json_path = generateSidecarJson(checksum);
    
    // 6. Queue for upload (if uploader is enabled)
    if (uploader_) {
        uploader_->enqueue(
            output_path_,
            json_path,
            config_.task_id,
            config_.factory,
            config_.device_id,
            checksum
        );
    }
    
    // 7. Invoke finish callback
    invokeFinishCallback();
}
```

### Recorder Node Integration

```cpp
// In recorder_node.cpp

RecorderNode::RecorderNode(const rclcpp::NodeOptions& options)
    : Node("axon_recorder", options) {
    
    // ... existing initialization ...
    
    // Initialize Edge Uploader if configured
    if (config_.upload.enabled) {
        UploaderConfig uploader_config;
        uploader_config.s3.endpoint_url = config_.upload.s3_endpoint;
        uploader_config.s3.bucket = config_.upload.s3_bucket;
        uploader_config.s3.region = config_.upload.s3_region;
        uploader_config.num_workers = config_.upload.num_workers;
        uploader_config.state_db_path = config_.upload.state_db_path;
        uploader_config.delete_after_upload = config_.upload.delete_after_upload;
        
        uploader_ = std::make_unique<EdgeUploader>(uploader_config);
        
        // start() performs crash recovery:
        // - Queries State DB for incomplete uploads from previous run
        // - Re-queues them for upload
        // - Then starts worker threads
        uploader_->start();
        
        RCLCPP_INFO(get_logger(), "Edge Uploader started with %d workers",
                    uploader_config.num_workers);
    }
}

RecorderNode::~RecorderNode() {
    if (uploader_) {
        uploader_->stop();
    }
}
```

## Configuration

### Extended YAML Configuration

```yaml
# /etc/axon/recorder_config.yaml

# ... existing recorder config ...

# Edge Upload Configuration
upload:
  enabled: true
  
  # S3 Configuration
  s3:
    endpoint_url: "https://edge-storage.example.com"  # Empty for AWS S3
    bucket: "axon-raw-data"
    region: "us-west-2"
    use_ssl: true
    verify_ssl: true
    
    # Multipart upload settings
    multipart_threshold_mb: 100
    multipart_chunk_size_mb: 50
    
  # Worker configuration
  num_workers: 2
  
  # Retry configuration
  retry:
    max_retries: 5
    initial_delay_ms: 1000
    max_delay_ms: 300000
    exponential_base: 2.0
    jitter: true
    
  # State persistence
  state_db_path: "/var/lib/axon/uploader_state.db"
  
  # Cleanup policy (files deleted immediately after successful upload)
  delete_after_upload: true
  failed_uploads_dir: "/data/failed_uploads/"
  min_free_space_gb: 20
  
  # Backpressure thresholds (for 4GB files)
  warn_pending_gb: 8      # ~2 files - emit warning
  alert_pending_gb: 20    # ~5 files - alert operator, consider stopping recording

# Edge Storage Lifecycle (managed by edge team, documented here for reference)
edge_lifecycle:
  retention_days: 7                    # Keep on edge for 7 days
  cloud_upload_after_qa: true          # Upload to cloud after QA passes
  delete_after_cloud_confirm: true     # Delete from edge after cloud copy verified
```

### Environment Variables

```bash
# S3-compatible credentials (works with any provider)
export AWS_ACCESS_KEY_ID="your-access-key"
export AWS_SECRET_ACCESS_KEY="your-secret-key"

# Optional: Override endpoint in environment (alternative to YAML config)
export AXON_S3_ENDPOINT="https://edge-storage.example.com"
export AXON_S3_BUCKET="axon-raw-data"
```

**Provider-specific examples**:

```bash
# MinIO (self-hosted)
export AWS_ACCESS_KEY_ID="minioadmin"
export AWS_SECRET_ACCESS_KEY="minioadmin"
export AXON_S3_ENDPOINT="http://minio.local:9000"

# Alibaba OSS
export AWS_ACCESS_KEY_ID="<your-oss-access-key>"
export AWS_SECRET_ACCESS_KEY="<your-oss-secret-key>"
export AXON_S3_ENDPOINT="https://oss-cn-shanghai.aliyuncs.com"

# Cloudflare R2
export AWS_ACCESS_KEY_ID="<your-r2-access-key>"
export AWS_SECRET_ACCESS_KEY="<your-r2-secret-key>"
export AXON_S3_ENDPOINT="https://<account-id>.r2.cloudflarestorage.com"
```

## Security Design

### Transport Security

- **TLS 1.3** required for all S3 connections
- **Certificate verification** enabled by default
- **No plaintext credentials** in logs or error messages

### Access Control (Per Provider)

**Principle**: Devices should only have write access to their own prefix (`{factory_id}/{device_id}/`).

#### MinIO Policy

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": ["s3:PutObject", "s3:AbortMultipartUpload"],
      "Resource": ["arn:aws:s3:::axon-raw-data/factory_01/device_01/*"]
    }
  ]
}
```

Create user with policy:
```bash
mc admin user add myminio device_user device_password
mc admin policy create myminio device-upload-policy policy.json
mc admin policy attach myminio device-upload-policy --user device_user
```

#### Ceph RADOS Gateway

```bash
radosgw-admin user create --uid=device_user --display-name="Device Uploader"
radosgw-admin caps add --uid=device_user --caps="buckets=read;objects=write"
```

#### AWS S3 (if used)

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": ["s3:PutObject", "s3:AbortMultipartUpload", "s3:ListMultipartUploadParts"],
      "Resource": ["arn:aws:s3:::axon-raw-data/${factory_id}/${device_id}/*"]
    }
  ]
}
```

### Object Metadata

Each uploaded object includes custom metadata for traceability (S3-compatible user metadata):

| Key | Example Value | Description |
|-----|---------------|-------------|
| `task-id` | `task_20251221_143052_abc123` | Task identifier |
| `device-id` | `robot_arm_001` | Device identifier |
| `factory-id` | `factory_shanghai_01` | Factory identifier |
| `uploader-version` | `1.0.0` | Uploader version |
| `checksum-sha256` | `a1b2c3d4...` | File checksum |

**Note**: All S3-compatible providers support user metadata via the `x-amz-meta-*` header prefix.

## Data Integrity

### Pre-Upload Validation

Before uploading, the recorder validates MCAP structural integrity:

```cpp
// mcap_validator.hpp

bool validateMcapStructure(const std::string& path) {
    // Level: Structural validation only (fast)
    // Does NOT decode message payloads (that's edge QA responsibility)
    
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;
    
    // 1. Check MCAP magic bytes (header)
    char magic[8];
    file.read(magic, 8);
    if (std::memcmp(magic, "\x89MCAP0\r\n", 8) != 0) {
        return false;  // Invalid header
    }
    
    // 2. Seek to end and verify footer magic
    file.seekg(-8, std::ios::end);
    file.read(magic, 8);
    if (std::memcmp(magic, "\x89MCAP0\r\n", 8) != 0) {
        return false;  // Truncated file or missing footer
    }
    
    // 3. Verify summary section is parseable
    //    (minio-cpp/mcap library handles this)
    mcap::McapReader reader;
    auto status = reader.open(path);
    if (!status.ok()) {
        return false;
    }
    
    auto summary = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
    if (!summary.ok()) {
        return false;  // Summary section corrupt
    }
    
    // Summary exists and is readable - file is structurally valid
    return true;
}
```

**Validation Levels**:

| Level | What it checks | When used |
|-------|----------------|-----------|
| **Structural (Device)** | Header, footer, summary section | Before upload |
| **Content QA (Edge)** | All messages, black image detection, sensor gaps | Post-upload pipeline |

**Corrupt file handling**: Drop immediately. Do not upload. No quarantine needed.

### Verification Flow

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ Local File      │────►│ Validate MCAP    │────►│ SHA-256 Checksum│
│ (MCAP)          │     │ (header/summary) │     │ (from sidecar)  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                       │                        │
        │                       ▼                        ▼
        │               ┌───────────────┐       ┌─────────────────┐
        │               │ Invalid?      │       │ Upload with     │
        │               │ → DROP FILE   │       │ x-amz-meta-*    │
        │               └───────────────┘       │ checksum header │
        │                                       └─────────────────┘
        │                                               │
        │                                               ▼
        │                                      ┌─────────────────┐
        │                                      │ Verify via HEAD │
        │                                      │ request metadata│
        │                                      └─────────────────┘
        │                                               │
        │                                      ┌────────┴────────┐
        │                                      │                 │
        │                                      ▼                 ▼
        │                               ┌───────────┐    ┌───────────┐
        │                               │  Match    │    │ Mismatch  │
        │                               │  → Done   │    │ → Re-upload│
        │                               └───────────┘    └───────────┘
        │
        └──────────────────────────────────────────────────────────────►
                                (JSON follows same upload path)
```

### Checksum Strategy

1. **Recorder validates MCAP structure** - header, footer, summary section check
2. **Recorder computes SHA-256** during finalization (stored in sidecar JSON)
3. **Uploader stores checksum in S3 metadata** (`x-amz-meta-checksum-sha256`)
4. **Post-upload verification** via HEAD request comparing stored metadata

**Why not ETag verification?**

For multipart uploads (files > 100MB), S3 ETag is **not** a simple MD5 hash. It's computed as:
```
ETag = MD5(MD5(part1) || MD5(part2) || ... || MD5(partN)) + "-" + N
```

This makes ETag comparison unreliable for integrity verification. Instead, we use custom metadata:

```cpp
UploadResult S3Client::uploadFile(...) {
    minio::s3::UploadObjectArgs args;
    args.bucket = config.bucket;
    args.object = s3_key;
    args.filename = local_path;
    
    // Store checksum in custom metadata (not ETag)
    args.user_metadata["checksum-sha256"] = checksum_sha256;
    
    // ... upload ...
}

bool S3Client::verifyUpload(const std::string& s3_key, 
                            const std::string& expected_checksum) {
    auto metadata = getObjectMetadata(s3_key);
    return metadata["checksum-sha256"] == expected_checksum;
}
```

## Monitoring & Observability

### Health Status API

```cpp
HealthStatus status = uploader.getHealthStatus();

// Returns:
// {
//   healthy: true,
//   message: "OK",
//   pending_count: 5,
//   failed_count: 0,
//   last_successful_upload: "2025-12-21T14:30:00Z"
// }
```

### Health Status File

Written periodically to `/var/run/axon/uploader_health.json`:

```json
{
  "status": "healthy",
  "timestamp": "2025-12-21T14:30:00Z",
  "uptime_sec": 3600,
  "metrics": {
    "files_pending": 5,
    "files_uploading": 2,
    "files_completed_last_hour": 42,
    "files_failed_last_hour": 1,
    "bytes_uploaded_last_hour": 10737418240,
    "current_upload_speed_mbps": 50.5
  },
  "last_upload": {
    "file": "task_20251221_143000.mcap",
    "timestamp": "2025-12-21T14:25:30Z",
    "status": "completed"
  },
  "errors": []
}
```

### Logging

Uses the recorder's existing logging infrastructure with structured fields:

```cpp
RCLCPP_INFO(logger_, "Upload started: %s -> %s [task_id=%s]", 
            local_path.c_str(), s3_key.c_str(), task_id.c_str());
RCLCPP_INFO(logger_, "Upload completed: %s (%.2f MB/s) [task_id=%s]", 
            s3_key.c_str(), speed_mbps, task_id.c_str());
RCLCPP_WARN(logger_, "Upload retry %d/%d: %s - %s [task_id=%s]", 
            retry_count, max_retries, s3_key.c_str(), error.c_str(), task_id.c_str());
RCLCPP_ERROR(logger_, "Upload failed: %s - %s [task_id=%s]", 
            s3_key.c_str(), error.c_str(), task_id.c_str());
```

### Failure Scenarios & Runbook

| Scenario | Symptoms | Automated Response | Operator Action |
|----------|----------|-------------------|-----------------|
| **Network unavailable** | `ConnectionTimeout` errors, queue growing | Retry with backoff | Check network, fix connectivity |
| **S3 credentials expired** | `AccessDenied` errors | Mark as non-retryable | Rotate credentials, restart recorder |
| **Disk full on device** | `enqueue()` fails, new recordings fail | Alert, stop accepting new uploads | Clear failed_uploads dir, expand storage |
| **S3 bucket full/quota** | `QuotaExceeded` errors | Retry (may be transient) | Contact edge team to expand quota |
| **Corrupt local file** | Checksum mismatch on upload | Move to failed_uploads | Investigate MCAP writer, check disk health |
| **SQLite DB corrupted** | State queries fail | Attempt recovery, fallback to file scan | Restore from backup or reinitialize |

**Alert Thresholds**:

| Metric | Warning | Critical |
|--------|---------|----------|
| Pending queue size | > 8 GB (2 files) | > 20 GB (5 files) |
| Upload failure rate | > 5% in 1 hour | > 20% in 1 hour |
| Time since last successful upload | > 10 minutes | > 30 minutes |

## Output Contract

### Uploader Guarantees

| Guarantee | Description |
|-----------|-------------|
| **At-least-once delivery** | Files will be uploaded at least once (may retry) |
| **Integrity verification** | SHA-256 checksum verified post-upload |
| **Ordered by file creation** | Files uploaded approximately in creation order |
| **Paired upload** | Both `.mcap` and `.json` uploaded atomically (see below) |

### File Pairing Strategy

To ensure downstream consumers see complete recording pairs:

```
Upload Order:
1. Upload .mcap file first (large, slow)
2. Upload .json sidecar last (small, fast) ← Signals "upload complete"
3. Mark both as complete when JSON succeeds

Downstream Detection:
- Dagster sensor watches for .json files
- JSON presence guarantees .mcap is already uploaded
- No additional existence check needed
```

**Rationale**: MCAP first, JSON last. When JSON appears in S3, the MCAP is guaranteed to be present. This simplifies downstream logic—no need to verify MCAP exists before processing.

```
Timeline:
────────────────────────────────────────────────────────────►
   │                    │                │
   │  MCAP uploading    │  MCAP done     │  JSON uploaded
   │  (4 min)           │                │  (instant)
   │                    │                │
   ▼                    ▼                ▼
   [Start]          [MCAP in S3]    [JSON in S3] ← Safe to process
```

**Failure Handling**:
- If MCAP fails: retry MCAP (JSON not uploaded yet)
- If MCAP succeeds but JSON fails: retry JSON only
- State DB tracks individual file status within a task

**Atomic Guarantee**: A task is only considered "uploaded" when both files are in S3. The JSON upload is the commit point.

### S3 Key Structure

```
s3://{bucket}/{factory_id}/{device_id}/{date}/{task_id}.{ext}

Example:
s3://axon-raw-data/factory_shanghai_01/robot_arm_001/2025-12-21/task_20251221_143052_abc123.mcap
s3://axon-raw-data/factory_shanghai_01/robot_arm_001/2025-12-21/task_20251221_143052_abc123.json
```

## Edge Storage Lifecycle

Data flows from device → edge → cloud with the following lifecycle:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              DATA LIFECYCLE                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│   DEVICE                    EDGE (7 days)                    CLOUD              │
│   ──────                    ────────────                    ─────              │
│                                                                                  │
│   ┌──────────┐  upload     ┌──────────────┐  QA pass      ┌──────────────┐    │
│   │Recording │───────────►│ S3 raw-data   │─────────────►│ Cloud Archive │    │
│   │finalize()│             │               │               │ (permanent)   │    │
│   └──────────┘             │ • MCAP        │               └──────────────┘    │
│        │                   │ • JSON        │                      │            │
│        │                   └──────────────┘                      │            │
│        │                          │                               │            │
│        ▼                          │ QA fail                       │            │
│   [Delete from device]            ▼                               │            │
│                            [Drop corrupt]                         │            │
│                                                                   │            │
│                            After cloud confirm:                   │            │
│                            [Delete from edge]◄────────────────────┘            │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘

Timeline:
─────────────────────────────────────────────────────────────────────────────────►
  │           │                │                    │                  │
  Record      Upload           QA Pipeline          Cloud Upload       Edge Delete
  (device)    (device→edge)    (edge compute)       (edge→cloud)       (edge)
  │           │                │                    │                  │
  0           +5 min           +1 hour              +2 hours           +7 days
```

### Lifecycle Rules

| Stage | Owner | Action | Trigger |
|-------|-------|--------|---------|
| **Device → Edge** | Device (Uploader) | Upload MCAP + JSON | `finalize()` completes |
| **Device cleanup** | Device (Uploader) | Delete local files | S3 upload confirmed |
| **Edge QA** | Edge (Dagster) | Validate content, run checks | JSON appears in S3 |
| **Edge → Cloud** | Edge (Dagster) | Upload to cloud archive | QA passes |
| **Edge cleanup** | Edge (S3 lifecycle) | Delete from edge | Cloud copy confirmed + 7 days |
| **Corrupt handling** | Both | Drop file | Validation fails |

### Edge Retention Policy

```
Edge S3 Bucket Lifecycle Configuration:

1. QA Passed recordings:
   - Keep for 7 days after cloud upload confirmed
   - Then delete automatically

2. QA Failed recordings:
   - Dropped immediately (no cloud upload)
   - Logged for debugging

3. Pending QA recordings:
   - Keep indefinitely until QA runs
   - Alert if queue grows too large
```

**Note**: The edge team owns the S3 lifecycle policies. The uploader's responsibility ends once data is confirmed in edge S3.

## Downstream Consumer Reference

The Edge Compute Layer (Dagster + Ray + Daft) consumes uploaded files. This section provides reference implementations for the edge team.

### Dagster Sensor for New Recordings

```python
from dagster import (
    sensor,
    RunRequest,
    SensorEvaluationContext,
    SkipReason,
    DefaultSensorStatus,
)
import boto3
import json

S3_BUCKET = "axon-raw-data"

@sensor(
    job=process_recording_job,
    minimum_interval_seconds=30,
    default_status=DefaultSensorStatus.RUNNING,
)
def new_recording_sensor(context: SensorEvaluationContext):
    """
    Sensor that detects new complete recordings in edge storage.
    
    JSON presence guarantees MCAP is already uploaded (JSON uploaded last).
    No need to check MCAP existence - it's guaranteed by upload order.
    """
    s3 = boto3.client('s3')
    
    # Get cursor (last processed timestamp)
    cursor = context.cursor or "1970-01-01T00:00:00Z"
    
    # List new JSON sidecars
    # JSON uploaded LAST = upload complete signal
    paginator = s3.get_paginator('list_objects_v2')
    new_recordings = []
    latest_timestamp = cursor
    
    for page in paginator.paginate(Bucket=S3_BUCKET):
        for obj in page.get('Contents', []):
            key = obj['Key']
            if not key.endswith('.json'):
                continue
                
            last_modified = obj['LastModified'].isoformat()
            if last_modified > cursor:
                new_recordings.append(key)
                latest_timestamp = max(latest_timestamp, last_modified)
    
    if not new_recordings:
        return SkipReason("No new recordings found")
    
    context.update_cursor(latest_timestamp)
    
    for json_key in new_recordings:
        response = s3.get_object(Bucket=S3_BUCKET, Key=json_key)
        metadata = json.loads(response['Body'].read())
        
        task_id = metadata['task']['task_id']
        mcap_key = json_key.replace('.json', '.mcap')
        
        # No need to verify MCAP exists - guaranteed by upload order
        # (MCAP uploaded first, JSON uploaded last)
        
        yield RunRequest(
            run_key=task_id,
            run_config={
                "ops": {
                    "process_recording": {
                        "config": {
                            "s3_bucket": S3_BUCKET,
                            "mcap_key": mcap_key,
                            "json_key": json_key,
                            "task_id": task_id,
                        }
                    }
                }
            },
        )
```

**Note**: For high-volume deployments (1000+ devices), consider using S3 Event Notifications → SQS instead of polling. This avoids expensive LIST operations.

### Daft Metadata Query

```python
import daft
from daft.io import IOConfig, S3Config

io_config = IOConfig(s3=S3Config(region_name="us-west-2"))
daft.set_planning_config(default_io_config=io_config)

def query_recordings_by_scene(scene: str, date: str):
    """Fast metadata scan using sidecar JSONs."""
    df = daft.read_json(f"s3://axon-raw-data/**/{date}/*.json")
    filtered = df.filter(df["task"]["scene"] == scene)
    return filtered.collect()
```

### Ray MCAP Processing

```python
import ray

@ray.remote(num_cpus=2)
def process_mcap(s3_bucket: str, mcap_key: str) -> dict:
    """Process a single MCAP file."""
    import boto3
    from mcap.reader import make_reader
    import io
    
    s3 = boto3.client('s3')
    response = s3.get_object(Bucket=s3_bucket, Key=mcap_key)
    data = response['Body'].read()
    
    with io.BytesIO(data) as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        
    return {
        "mcap_key": mcap_key,
        "message_count": summary.statistics.message_count,
        "schema_count": len(summary.schemas),
    }
```

## Build Integration

### CMakeLists.txt Addition

```cmake
# Find minio-cpp (or use FetchContent)
find_package(miniocpp QUIET)

if(NOT miniocpp_FOUND)
    # Fetch minio-cpp if not installed
    include(FetchContent)
    FetchContent_Declare(
        miniocpp
        GIT_REPOSITORY https://github.com/minio/minio-cpp.git
        GIT_TAG        v0.3.0
    )
    FetchContent_MakeAvailable(miniocpp)
endif()

# Edge Uploader library
add_library(edge_uploader
    src/edge_uploader/edge_uploader.cpp
    src/edge_uploader/upload_queue.cpp
    src/edge_uploader/s3_client.cpp
    src/edge_uploader/upload_state_manager.cpp
    src/edge_uploader/retry_handler.cpp
)

target_link_libraries(edge_uploader
    miniocpp::miniocpp
    sqlite3
    ${catkin_LIBRARIES}  # or ament for ROS2
)

target_include_directories(edge_uploader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link to recorder library
target_link_libraries(axon_recorder_lib
    edge_uploader
)
```

### Dependencies

```bash
# Ubuntu/Debian - minio-cpp dependencies
sudo apt-get install libcurl4-openssl-dev libssl-dev libsqlite3-dev

# Optional: Install minio-cpp system-wide
git clone https://github.com/minio/minio-cpp.git
cd minio-cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Supported S3-Compatible Providers

minio-cpp works with any S3-compatible storage:

| Provider | Endpoint Example |
|----------|------------------|
| AWS S3 | `https://s3.amazonaws.com` |
| MinIO | `https://play.min.io` or self-hosted |
| Ceph RADOS Gateway | `https://ceph-rgw.example.com` |
| Alibaba OSS | `https://oss-cn-hangzhou.aliyuncs.com` |
| Cloudflare R2 | `https://<account>.r2.cloudflarestorage.com` |
| DigitalOcean Spaces | `https://nyc3.digitaloceanspaces.com` |
| Backblaze B2 | `https://s3.us-west-000.backblazeb2.com` |

## Development Environment (Mock S3)

For development and testing without an edge server, use **MinIO on tmpfs** - a RAM-based storage that provides S3-compatible API with built-in metrics.

### Setup MinIO on tmpfs

```bash
# 1. Create tmpfs mount (RAM disk) - data is discarded on restart
sudo mkdir -p /mnt/minio-tmpfs
sudo mount -t tmpfs -o size=2G tmpfs /mnt/minio-tmpfs

# 2. Run MinIO with Prometheus metrics enabled
docker run -d \
  --name minio-dev \
  -p 9000:9000 \
  -p 9001:9001 \
  -v /mnt/minio-tmpfs:/data \
  -e "MINIO_ROOT_USER=minioadmin" \
  -e "MINIO_ROOT_PASSWORD=minioadmin" \
  -e "MINIO_PROMETHEUS_AUTH_TYPE=public" \
  quay.io/minio/minio server /data --console-address ":9001"

# 3. Create test bucket
docker run --rm --network host \
  -e MC_HOST_local=http://minioadmin:minioadmin@localhost:9000 \
  quay.io/minio/mc mb local/axon-raw-data
```

### Why tmpfs?

| Aspect | tmpfs (RAM) | Disk Storage |
|--------|-------------|--------------|
| Write speed | ~GB/s | ~100-500 MB/s |
| Data persistence | Discarded on restart | Persistent |
| Use case | Testing, benchmarking | Production |
| Disk space used | None | Accumulates |

### Access Metrics

MinIO exposes Prometheus metrics for tracking upload activity:

```bash
# View metrics
curl http://localhost:9000/minio/v2/metrics/cluster

# Key metrics:
# - minio_s3_requests_total{api="putobject"} - PUT request count
# - minio_s3_traffic_received_bytes          - Total bytes received
# - minio_s3_requests_waiting_total          - Requests in queue
```

### Configure Uploader for Development

```yaml
# /etc/axon/recorder_config.yaml (development)
upload:
  enabled: true
  s3:
    endpoint_url: "http://localhost:9000"
    bucket: "axon-raw-data"
    region: "us-east-1"
    access_key: "minioadmin"
    secret_key: "minioadmin"
    use_ssl: false
    verify_ssl: false
```

### MinIO Web Console

Access the MinIO console at `http://localhost:9001` to:
- Browse uploaded files
- View bucket statistics
- Monitor real-time metrics

### Cleanup

```bash
# Stop and remove container
docker stop minio-dev && docker rm minio-dev

# Unmount tmpfs (frees RAM)
sudo umount /mnt/minio-tmpfs
```

## Testing Strategy

### Unit Tests

```cpp
TEST(UploadQueueTest, EnqueueDequeue) {
    UploadQueue queue;
    UploadItem item{.mcap_path = "/tmp/test.mcap", .task_id = "task_001"};
    
    ASSERT_TRUE(queue.enqueue(item));
    auto dequeued = queue.dequeue();
    ASSERT_TRUE(dequeued.has_value());
    EXPECT_EQ(dequeued->task_id, "task_001");
}

TEST(RetryHandlerTest, ExponentialBackoff) {
    RetryHandler handler;
    
    auto delay0 = handler.getDelay(0);
    auto delay1 = handler.getDelay(1);
    auto delay2 = handler.getDelay(2);
    
    // Delays should increase exponentially (with jitter)
    EXPECT_GT(delay1.count(), delay0.count());
    EXPECT_GT(delay2.count(), delay1.count());
}

TEST(S3ClientTest, ConstructS3Key) {
    EdgeUploader uploader(config);
    
    auto key = uploader.constructS3Key(
        "factory_01", "robot_01", "task_001", "mcap"
    );
    
    // Key should follow pattern: factory/device/date/task.ext
    EXPECT_THAT(key, testing::MatchesRegex(
        R"(factory_01/robot_01/\d{4}-\d{2}-\d{2}/task_001\.mcap)"
    ));
}
```

### Integration Tests

```cpp
TEST(EdgeUploaderIntegrationTest, UploadAndVerify) {
    // Requires MinIO on tmpfs (see "Development Environment" section)
    UploaderConfig config;
    config.s3.endpoint_url = "http://localhost:9000";
    config.s3.bucket = "axon-raw-data";
    config.s3.access_key = "minioadmin";
    config.s3.secret_key = "minioadmin";
    
    EdgeUploader uploader(config);
    uploader.start();
    
    // Create test file
    std::string test_file = createTestMcapFile();
    std::string test_json = createTestSidecarJson();
    
    // Enqueue upload
    uploader.enqueue(test_file, test_json, "test_task", "factory", "device", "checksum");
    
    // Wait for upload to complete
    waitForUploadComplete(uploader, "test_task");
    
    // Verify file exists in S3
    S3Client s3(config.s3);
    EXPECT_TRUE(s3.objectExists("factory/device/2025-12-21/test_task.mcap"));
    
    uploader.stop();
}
```

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-21 | - | Initial design (Python) |
| 1.1 | 2025-12-21 | - | Revised to C++ implementation; clarified device/edge boundary |
| 1.2 | 2025-12-21 | - | Added SLOs, performance requirements, failure runbook, file pairing strategy |
| 1.3 | 2025-12-22 | - | Design review updates: reversed upload order (MCAP→JSON), fixed checksum verification (custom metadata vs ETag), added MCAP structural validation, SQLite WAL mode, 7-day edge lifecycle |

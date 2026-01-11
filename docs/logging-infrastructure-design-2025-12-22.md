# Logging Infrastructure Design

## Overview

This document defines a unified logging infrastructure for `axon_recorder`, replacing the current ad-hoc mix of `std::cerr`, `std::cout`, and ROS logging with a structured, configurable, and observable logging system. The design leverages **Boost.Log** for production-grade async logging across all components.

**Key Design Decision**: Use Boost.Log everywhere (both `axon_recorder` and `axon_mcap`). Since `axon_mcap` is only a wrapper for Foxglove MCAP and is exclusively used by `axon_recorder`, there's no need for a separate callback-based logging system.

## Problem Statement

Current logging in `axon_recorder` and `axon_mcap` suffers from several issues:

| Issue | Impact | Current State |
|-------|--------|---------------|
| Inconsistent output | Debugging difficulty | Mix of `std::cerr`, `std::cout`, ROS_INFO/RCLCPP_INFO |
| No log levels control | Cannot adjust verbosity at runtime | Hard-coded output statements |
| No file logging | Lost logs after session | Only console output |
| No structured format | Cannot parse/analyze logs | Free-form text messages |
| Missing context | Cannot correlate events | No task_id, device_id in logs |
| No log rotation | Disk exhaustion risk | N/A (no file logging) |
| No library logging | Hidden errors in MCAP layer | Errors only in `get_last_error()` |

**Affected Components**:

| Component | Location | ROS Dependency | Current Logging |
|-----------|----------|----------------|-----------------|
| `axon_recorder` | `middlewares/axon_recorder/` | Yes (ROS1/ROS2) | `std::cerr`, ROS macros |
| `axon_mcap` | `core/axon_mcap/` | No (but only used by axon_recorder) | `last_error_` only, no logging |

**Current State Analysis**:

```cpp
// recorder_node.cpp - Uses std::cerr
std::cerr << "[axon_recorder] Entering spin()..." << std::endl;
std::cerr << "[axon_recorder] Recording stopped" << std::endl;

// ros_interface.cpp - Uses ROS logging via interface
ROS_INFO("%s", message.c_str());      // ROS1
RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());  // ROS2

// recording_session.cpp - Uses std::cerr for errors
std::cerr << "[RecordingSession] Failed to open: " << writer_->get_last_error() << std::endl;

// Tests - Uses std::cout
std::cout << "=== Publishing Statistics ===\n";

// core/axon_mcap/mcap_writer_wrapper.cpp - No logging, errors stored in string
if (!status.ok()) {
    last_error_ = "Failed to write message: " + status.message;
    return false;  // Error not logged!
}
```

**Solution**: A unified logging layer that provides:
1. Consistent API across all components
2. Multiple output backends (console, file, ROS)
3. Structured logging with JSON support
4. Context injection (task_id, device_id, etc.)
5. Configurable log levels and rotation

## Design Goals

1. **Unified API** - Single logging interface (`AXON_LOG_*` macros) for all components
2. **Zero Overhead** - No impact on recording hot path (async sinks, compile-time DEBUG elimination)
3. **Structured Output** - JSON format for log analysis tools
4. **Context-Aware** - Automatic injection of task/device metadata via scoped attributes
5. **Flexible Backends** - Console, file, ROS via Boost.Log sink architecture
6. **Production-Ready** - Async I/O, bounded queues, time-based rotation (via Boost.Log)
7. **ROS Compatible** - Custom sink bridges to ROS logging infrastructure
8. **Simplicity** - Single logging implementation (Boost.Log) across all components
9. **Log Sampling** - Built-in macros for count-based and time-based sampling of high-frequency events
10. **Runtime Configurable** - YAML config file + environment variable overrides for deployment flexibility

## Technology Choice: Boost.Log

After evaluating custom implementation vs Boost.Log, we chose **Boost.Log** for `axon_recorder`:

| Requirement | Boost.Log | Custom Implementation |
|-------------|-----------|----------------------|
| Async file I/O | ✅ `asynchronous_sink` built-in | ❌ Would need to implement |
| Bounded queue (backpressure) | ✅ `bounded_fifo_queue` with drop/block | ❌ Not designed |
| Time-based rotation | ✅ `rotation_at_time_point` | ❌ Only size-based |
| Battle-tested thread safety | ✅ Production-proven | ⚠️ Needs extensive testing |
| Maintenance burden | ✅ Community-maintained | ❌ ~500 lines to maintain |

**Trade-off**: Boost.Log adds dependency (~5-10MB), but eliminates critical issues (blocking I/O, no async, no bounded queue) identified in design review.

**Note**: Since `axon_mcap` is only a wrapper for Foxglove MCAP and is exclusively used by `axon_recorder`, it uses the same Boost.Log infrastructure. No separate logging system needed.

## Architecture

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│                   Logging Infrastructure Architecture (Boost.Log)                 │
├──────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│   middlewares/axon_recorder/                                                             │
│   ┌─────────────────────────────────────────────────────────────────────────┐    │
│   │  RecorderNode         RecordingSession        WorkerThreadPool          │    │
│   │       │                     │                       │                    │    │
│   │       │    AXON_LOG_INFO("Recording started") << kv("task_id", id);     │    │
│   │       │                     │                       │                    │    │
│   │       └─────────────────────┴───────────────────────┘                    │    │
│   └─────────────────────────────┬────────────────────────────────────────────┘    │
│                                 │                                                 │
│   core/axon_mcap/ (wrapper for Foxglove MCAP)                                     │
│   ┌─────────────────────────────┼───────────────────────────────────────────┐    │
│   │  McapWriterWrapper          │                                            │    │
│   │       │                     │                                            │    │
│   │       │    AXON_LOG_ERROR("Write failed") << kv("error", status);       │    │
│   │       │                     │                                            │    │
│   │       └─────────────────────┘                                            │    │
│   └─────────────────────────────┬────────────────────────────────────────────┘    │
│                                 │                                                 │
│                                 ▼                                                 │
│   ┌─────────────────────────────────────────────────────────────────────────┐    │
│   │                      Boost.Log Core                                      │    │
│   │  ┌──────────────────────────────────────────────────────────────────┐   │    │
│   │  │  • Severity filtering (debug/info/warn/error/fatal)              │   │    │
│   │  │  • Scoped attributes (TaskID, DeviceID)                          │   │    │
│   │  │  • Thread-safe record dispatch                                   │   │    │
│   │  │  • Formatter chain (JSON, text)                                  │   │    │
│   │  └──────────────────────────────────────────────────────────────────┘   │    │
│   └─────────────────────────────┬────────────────────────────────────────────┘    │
│                                 │                                                 │
│            ┌────────────────────┼────────────────────┐                           │
│            │                    │                    │                           │
│            ▼                    ▼                    ▼                           │
│   ┌─────────────────────────────────────────────────────────────────────────┐    │
│   │                  Boost.Log Sinks (All Async)                             │    │
│   │                                                                          │    │
│   │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐       │    │
│   │  │ asynchronous_sink│  │ asynchronous_sink│  │ synchronous_sink │       │    │
│   │  │ + text_ostream   │  │ + text_file      │  │ + RosBackend     │       │    │
│   │  │                  │  │                  │  │                  │       │    │
│   │  │ • stderr/stdout  │  │ • JSON format    │  │ • ROS_INFO       │       │    │
│   │  │ • Colored output │  │ • Size rotation  │  │ • RCLCPP_INFO    │       │    │
│   │  │ • bounded_fifo   │  │ • Time rotation  │  │ • Bridge to ROS  │       │    │
│   │  │   (drop overflow)│  │ • bounded_fifo   │  │                  │       │    │
│   │  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘       │    │
│   │           │                     │                     │                  │    │
│   └───────────┼─────────────────────┼─────────────────────┼──────────────────┘    │
│               │                     │                     │                       │
│               ▼                     ▼                     ▼                       │
│   ┌──────────────────┐  ┌────────────────────────┐  ┌──────────────────────┐     │
│   │     Terminal     │  │  /var/log/axon/        │  │   ROS Log System     │     │
│   │                  │  │  recorder_YYYYMMDD.log │  │   (rosout/console)   │     │
│   │                  │  │  (auto-rotated)        │  │                      │     │
│   └──────────────────┘  └────────────────────────┘  └──────────────────────┘     │
│                                                                                   │
└──────────────────────────────────────────────────────────────────────────────────┘
```

**Key Design Decision**: Single Boost.Log implementation across all components. No separate logging systems, no bridges needed.

## Log Levels

| Level | Numeric | Usage | Production Default |
|-------|---------|-------|-------------------|
| `DEBUG` | 0 | Detailed debugging, per-message tracing | Off |
| `INFO` | 1 | Normal operational messages | On |
| `WARN` | 2 | Unexpected but recoverable situations | On |
| `ERROR` | 3 | Errors requiring attention | On |

**Level Selection Guidelines**:

```cpp
// DEBUG: Detailed internal state, per-message events
AXON_LOG_DEBUG("Processing message", "topic", topic, "size", msg.size());

// INFO: Normal operational milestones
AXON_LOG_INFO("Recording started", "task_id", task_id, "topics", topics.size());

// WARN: Unexpected but handled situations
AXON_LOG_WARN("Queue overflow, dropping messages", "topic", topic, "dropped", count);

// ERROR: Failures requiring operator attention
AXON_LOG_ERROR("Failed to write MCAP chunk", "error", error_msg, "bytes_lost", bytes);
```

## Log Format Specification

### Text Format (Console)

Human-readable format for development and console output:

```
[2025-12-22T14:30:52.123456Z] [INFO] [axon_recorder] Recording started | task_id=task_001 device_id=robot_arm_001 topics=5
[2025-12-22T14:30:52.234567Z] [DEBUG] [recording_session] Channel registered | topic=/camera/image_raw schema_id=1
[2025-12-22T14:30:55.345678Z] [WARN] [worker_pool] Queue depth high | topic=/camera/image_raw depth=1800 threshold=2000
[2025-12-22T14:31:00.456789Z] [ERROR] [mcap_writer] Write failed | error="disk full" bytes_pending=104857600
```

Format: `[{timestamp}] [{level}] [{component}] {message} | {key}={value} ...`

### JSON Format (File)

Structured format for log aggregation and analysis:

```json
{"ts":"2025-12-22T14:30:52.123456Z","level":"INFO","component":"axon_recorder","msg":"Recording started","task_id":"task_001","device_id":"robot_arm_001","topics":5}
{"ts":"2025-12-22T14:30:52.234567Z","level":"DEBUG","component":"recording_session","msg":"Channel registered","topic":"/camera/image_raw","schema_id":1}
{"ts":"2025-12-22T14:30:55.345678Z","level":"WARN","component":"worker_pool","msg":"Queue depth high","topic":"/camera/image_raw","depth":1800,"threshold":2000}
{"ts":"2025-12-22T14:31:00.456789Z","level":"ERROR","component":"mcap_writer","msg":"Write failed","error":"disk full","bytes_pending":104857600}
```

**JSON Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `ts` | string | ISO 8601 timestamp with microsecond precision |
| `level` | string | Log level (DEBUG, INFO, WARN, ERROR) |
| `component` | string | Source component name |
| `msg` | string | Log message |
| `task_id` | string | Current task ID (if in recording session) |
| `device_id` | string | Device identifier |
| `*` | any | Additional context key-value pairs |

## Component Design

### 1. Boost.Log Severity Levels

Custom severity levels matching our requirements.

```cpp
// axon_log_severity.hpp
#pragma once

#include <boost/log/expressions/keyword.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <ostream>

namespace axon {
namespace logging {

/**
 * Severity levels for axon logging.
 * Maps to standard log levels with FATAL for unrecoverable errors.
 */
enum class severity_level {
    debug = 0,
    info = 1,
    warn = 2,
    error = 3,
    fatal = 4
};

// Output operator for formatting
inline std::ostream& operator<<(std::ostream& strm, severity_level level) {
    static const char* strings[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
    if (static_cast<size_t>(level) < sizeof(strings) / sizeof(*strings))
        strm << strings[static_cast<size_t>(level)];
    else
        strm << static_cast<int>(level);
    return strm;
}

// Boost.Log keyword for severity
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level)
BOOST_LOG_ATTRIBUTE_KEYWORD(component, "Component", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(task_id, "TaskID", std::string)
BOOST_LOG_ATTRIBUTE_KEYWORD(device_id, "DeviceID", std::string)

}  // namespace logging
}  // namespace axon
```

### 2. Convenience Macros (Boost.Log Wrapper)

Zero-overhead macros that wrap Boost.Log with a clean API.

```cpp
// axon_log_macros.hpp
#pragma once

#include "axon_log_severity.hpp"
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>

namespace axon {
namespace logging {

// Global severity logger
typedef boost::log::sources::severity_logger<severity_level> logger_type;
logger_type& get_logger();

// Key-value helper for structured logging
template<typename T>
inline boost::log::aux::add_value_manip<T> kv(const char* name, const T& value) {
    return boost::log::add_value(name, value);
}

}  // namespace logging
}  // namespace axon

// Component name should be defined per-file
#ifndef AXON_LOG_COMPONENT
#define AXON_LOG_COMPONENT "axon_recorder"
#endif

// Compile-time debug toggle
#ifdef NDEBUG
#define AXON_LOG_ENABLE_DEBUG 0
#else
#define AXON_LOG_ENABLE_DEBUG 1
#endif

// Main logging macros - stream-based API
#define AXON_LOG_DEBUG(msg) \
    do { \
        if (AXON_LOG_ENABLE_DEBUG) { \
            BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::debug) \
                << ::axon::logging::kv("Component", AXON_LOG_COMPONENT) << msg; \
        } \
    } while(0)

#define AXON_LOG_INFO(msg) \
    BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::info) \
        << ::axon::logging::kv("Component", AXON_LOG_COMPONENT) << msg

#define AXON_LOG_WARN(msg) \
    BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::warn) \
        << ::axon::logging::kv("Component", AXON_LOG_COMPONENT) << msg

#define AXON_LOG_ERROR(msg) \
    BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::error) \
        << ::axon::logging::kv("Component", AXON_LOG_COMPONENT) << msg

#define AXON_LOG_FATAL(msg) \
    BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::fatal) \
        << ::axon::logging::kv("Component", AXON_LOG_COMPONENT) << msg

// Context management using scoped attributes
#define AXON_LOG_SCOPED_CONTEXT(task_id_val, device_id_val) \
    BOOST_LOG_SCOPED_THREAD_ATTR("TaskID", \
        boost::log::attributes::constant<std::string>(task_id_val)); \
    BOOST_LOG_SCOPED_THREAD_ATTR("DeviceID", \
        boost::log::attributes::constant<std::string>(device_id_val))

// =============================================================================
// Log Sampling Macros
// =============================================================================

// Count-based sampling: Log every Nth occurrence
// Useful for high-frequency loops where you want periodic status updates
#define AXON_LOG_INFO_EVERY_N(n, msg) \
    do { \
        static std::atomic<uint64_t> _axon_log_counter{0}; \
        if ((++_axon_log_counter % (n)) == 1) { \
            AXON_LOG_INFO(msg); \
        } \
    } while(0)

// Time-based throttling: Log at most once per specified interval (seconds)
// Useful for error conditions that may repeat rapidly
#define AXON_LOG_WARN_THROTTLE(interval_sec, msg) \
    do { \
        static std::chrono::steady_clock::time_point _axon_last_log_time{}; \
        static std::mutex _axon_throttle_mutex; \
        auto _axon_now = std::chrono::steady_clock::now(); \
        bool _axon_should_log = false; \
        { \
            std::lock_guard<std::mutex> _axon_lock(_axon_throttle_mutex); \
            if (_axon_now - _axon_last_log_time >= std::chrono::duration<double>(interval_sec)) { \
                _axon_last_log_time = _axon_now; \
                _axon_should_log = true; \
            } \
        } \
        if (_axon_should_log) { \
            AXON_LOG_WARN(msg); \
        } \
    } while(0)

// Both macro families available for DEBUG, INFO, WARN, ERROR levels
```

**Usage Examples**:

```cpp
// In recorder_node.cpp
#define AXON_LOG_COMPONENT "recorder_node"
#include "axon_log_macros.hpp"

void RecorderNode::start_recording() {
    // Set context for this recording session (scoped - auto-cleared on function exit)
    AXON_LOG_SCOPED_CONTEXT(task_config_.task_id, config_.device.id);
    
    AXON_LOG_INFO("Recording started" 
        << kv("scene", task_config_.scene)
        << kv("topics", topic_configs_.size()));
}

// In recording_session.cpp
#define AXON_LOG_COMPONENT "recording_session"
#include "axon_log_macros.hpp"

bool RecordingSession::open(const std::string& path) {
    AXON_LOG_DEBUG("Opening MCAP file" << kv("path", path));
    
    if (!writer_->open(path)) {
        AXON_LOG_ERROR("Failed to open MCAP file"
            << kv("path", path)
            << kv("error", writer_->get_last_error()));
        return false;
    }
    
    AXON_LOG_INFO("MCAP file opened" << kv("path", path));
    return true;
}

// Log sampling for high-frequency events
void WorkerThread::process_messages() {
    for (int i = 0; i < 1000000; ++i) {
        // Log every 10000th message (count-based sampling)
        AXON_LOG_DEBUG_EVERY_N(10000, "Processing message" << kv("iteration", i));
        
        // ... process message ...
    }
}

// Rate-limited error logging
void RecorderNode::check_disk_space() {
    if (is_disk_full()) {
        // Log at most once per 10 seconds (time-based throttling)
        AXON_LOG_ERROR_THROTTLE(10.0, "Disk full, cannot write" 
            << kv("free_mb", get_free_space_mb()));
    }
}
```

### 3. Console Sink (Boost.Log Async)

Async console output with colored text formatting.

```cpp
// axon_console_sink.hpp
#pragma once

#include "axon_log_severity.hpp"
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sinks/bounded_fifo_queue.hpp>
#include <boost/log/sinks/drop_on_overflow.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>

namespace axon {
namespace logging {

// Async console sink with bounded queue (drops on overflow to prevent blocking)
typedef boost::log::sinks::asynchronous_sink<
    boost::log::sinks::text_ostream_backend,
    boost::log::sinks::bounded_fifo_queue<
        1000,  // Max 1000 records in queue
        boost::log::sinks::drop_on_overflow
    >
> async_console_sink_t;

/**
 * Create async console sink with colored output.
 * Uses bounded queue to prevent memory explosion under high load.
 */
boost::shared_ptr<async_console_sink_t> create_console_sink(
    severity_level min_level = severity_level::info,
    bool use_colors = true
);

}  // namespace logging
}  // namespace axon
```

**Implementation**:

```cpp
// axon_console_sink.cpp
#include "axon_console_sink.hpp"
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/core/null_deleter.hpp>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;

// Color codes for terminal output
const char* get_color(severity_level level) {
    switch (level) {
        case severity_level::debug: return "\033[36m";  // Cyan
        case severity_level::info:  return "\033[32m";  // Green
        case severity_level::warn:  return "\033[33m";  // Yellow
        case severity_level::error: return "\033[31m";  // Red
        case severity_level::fatal: return "\033[35m";  // Magenta
        default: return "";
    }
}

boost::shared_ptr<async_console_sink_t> create_console_sink(
    severity_level min_level,
    bool use_colors
) {
    // Create backend
    auto backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
    backend->add_stream(
        boost::shared_ptr<std::ostream>(&std::clog, boost::null_deleter()));
    backend->auto_flush(true);
    
    // Create async sink with bounded queue
    auto sink = boost::make_shared<async_console_sink_t>(backend);
    
    // Set filter
    sink->set_filter(severity >= min_level);
    
    // Set formatter with optional colors
    if (use_colors) {
        sink->set_formatter([](boost::log::record_view const& rec, 
                               boost::log::formatting_ostream& strm) {
            auto sev = rec[severity];
            strm << "[" << rec[expr::attr<boost::posix_time::ptime>("TimeStamp")] << "] ";
            strm << get_color(*sev) << "[" << *sev << "]\033[0m ";
            strm << "[" << rec[expr::attr<std::string>("Component")] << "] ";
            strm << rec[expr::smessage];
        });
    } else {
        sink->set_formatter(
            expr::stream
                << "[" << expr::attr<boost::posix_time::ptime>("TimeStamp") << "] "
                << "[" << severity << "] "
                << "[" << expr::attr<std::string>("Component") << "] "
                << expr::smessage
        );
    }
    
    return sink;
}

}  // namespace logging
}  // namespace axon
```

### 4. File Sink with Rotation (Boost.Log Async)

Async JSON file output with size and time-based rotation.

```cpp
// axon_file_sink.hpp
#pragma once

#include "axon_log_severity.hpp"
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/bounded_fifo_queue.hpp>
#include <boost/log/sinks/drop_on_overflow.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <string>

namespace axon {
namespace logging {

// Async file sink with bounded queue
typedef boost::log::sinks::asynchronous_sink<
    boost::log::sinks::text_file_backend,
    boost::log::sinks::bounded_fifo_queue<
        5000,  // Larger queue for file sink (slower I/O)
        boost::log::sinks::drop_on_overflow
    >
> async_file_sink_t;

struct FileSinkConfig {
    std::string directory = "/var/log/axon";
    std::string file_pattern = "recorder_%Y%m%d_%H%M%S.log";
    uint64_t rotation_size_mb = 100;      // Rotate at 100MB
    bool rotate_at_midnight = true;       // Also rotate daily
    int max_files = 10;                   // Keep 10 rotated files
    bool format_json = true;              // JSON format for log aggregation
};

/**
 * Create async file sink with rotation.
 * - Size-based rotation (default 100MB)
 * - Time-based rotation (midnight)
 * - Bounded queue prevents memory explosion
 * - JSON format for log aggregation tools
 */
boost::shared_ptr<async_file_sink_t> create_file_sink(
    const FileSinkConfig& config,
    severity_level min_level = severity_level::debug
);

}  // namespace logging
}  // namespace axon
```

**Implementation**:

```cpp
// axon_file_sink.cpp
#include "axon_file_sink.hpp"
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;

// JSON formatter for file output
void json_formatter(boost::log::record_view const& rec, 
                    boost::log::formatting_ostream& strm) {
    // Escape helper
    auto escape = [](const std::string& s) {
        std::string result;
        for (char c : s) {
            switch (c) {
                case '"':  result += "\\\""; break;
                case '\\': result += "\\\\"; break;
                case '\n': result += "\\n"; break;
                case '\r': result += "\\r"; break;
                case '\t': result += "\\t"; break;
                default:   result += c;
            }
        }
        return result;
    };
    
    strm << "{";
    strm << "\"ts\":\"" << rec[expr::attr<boost::posix_time::ptime>("TimeStamp")] << "\",";
    strm << "\"level\":\"" << rec[severity] << "\",";
    strm << "\"component\":\"" << escape(*rec[expr::attr<std::string>("Component")]) << "\",";
    strm << "\"msg\":\"" << escape(rec[expr::smessage].get()) << "\"";
    
    // Optional context attributes
    if (auto tid = rec[expr::attr<std::string>("TaskID")]) {
        strm << ",\"task_id\":\"" << escape(*tid) << "\"";
    }
    if (auto did = rec[expr::attr<std::string>("DeviceID")]) {
        strm << ",\"device_id\":\"" << escape(*did) << "\"";
    }
    
    strm << "}";
}

boost::shared_ptr<async_file_sink_t> create_file_sink(
    const FileSinkConfig& config,
    severity_level min_level
) {
    namespace sinks = boost::log::sinks;
    
    // Create backend with rotation
    auto backend = boost::make_shared<sinks::text_file_backend>(
        keywords::file_name = config.directory + "/" + config.file_pattern,
        keywords::rotation_size = config.rotation_size_mb * 1024 * 1024,
        keywords::time_based_rotation = config.rotate_at_midnight 
            ? sinks::file::rotation_at_time_point(0, 0, 0)  // Midnight
            : sinks::file::rotation_at_time_point(),
        keywords::auto_flush = true,
        keywords::enable_final_rotation = false  // Prevent crash on termination
    );
    
    // Set up file collector for rotation management
    backend->set_file_collector(sinks::file::make_collector(
        keywords::target = config.directory,
        keywords::max_files = config.max_files
    ));
    
    // Scan for existing files on startup
    backend->scan_for_files();
    
    // Create async sink
    auto sink = boost::make_shared<async_file_sink_t>(backend);
    
    // Set filter
    sink->set_filter(severity >= min_level);
    
    // Set formatter
    if (config.format_json) {
        sink->set_formatter(&json_formatter);
    } else {
        sink->set_formatter(
            expr::stream
                << "[" << expr::attr<boost::posix_time::ptime>("TimeStamp") << "] "
                << "[" << severity << "] "
                << "[" << expr::attr<std::string>("Component") << "] "
                << expr::smessage
        );
    }
    
    return sink;
}

}  // namespace logging
}  // namespace axon
```

### 5. ROS Sink (Custom Boost.Log Backend)

Custom Boost.Log backend that bridges to ROS logging.

```cpp
// axon_ros_sink.hpp
#pragma once

#include "axon_log_severity.hpp"
#include "ros_interface.hpp"
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace axon {
namespace logging {

/**
 * Custom Boost.Log backend that forwards to ROS logging.
 * Uses synchronous sink (ROS logging is fast, no async needed).
 */
class ros_backend : 
    public boost::log::sinks::basic_formatted_sink_backend<
        char, 
        boost::log::sinks::synchronized_feeding
    > 
{
public:
    explicit ros_backend(recorder::RosInterface* ros_interface);
    
    // Called by Boost.Log for each record
    void consume(boost::log::record_view const& rec, string_type const& formatted);
    
private:
    recorder::RosInterface* ros_interface_;
};

// Sync sink type for ROS (no async needed)
typedef boost::log::sinks::synchronous_sink<ros_backend> ros_sink_t;

/**
 * Create ROS sink that bridges to ROS logging.
 * Logs appear in rqt_console, rosout, etc.
 */
boost::shared_ptr<ros_sink_t> create_ros_sink(
    recorder::RosInterface* ros_interface,
    severity_level min_level = severity_level::info
);

}  // namespace logging
}  // namespace axon
```

**Implementation**:

```cpp
// axon_ros_sink.cpp
#include "axon_ros_sink.hpp"
#include <boost/log/expressions.hpp>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;

ros_backend::ros_backend(recorder::RosInterface* ros_interface)
    : ros_interface_(ros_interface) {}

void ros_backend::consume(boost::log::record_view const& rec, 
                          string_type const& formatted) {
    if (!ros_interface_) return;
    
    auto sev = rec[severity];
    if (!sev) return;
    
    switch (*sev) {
        case severity_level::debug:
            ros_interface_->log_debug(formatted);
            break;
        case severity_level::info:
            ros_interface_->log_info(formatted);
            break;
        case severity_level::warn:
            ros_interface_->log_warn(formatted);
            break;
        case severity_level::error:
        case severity_level::fatal:
            ros_interface_->log_error(formatted);
            break;
    }
}

boost::shared_ptr<ros_sink_t> create_ros_sink(
    recorder::RosInterface* ros_interface,
    severity_level min_level
) {
    auto backend = boost::make_shared<ros_backend>(ros_interface);
    auto sink = boost::make_shared<ros_sink_t>(backend);
    
    // Set filter
    sink->set_filter(severity >= min_level);
    
    // Simple formatter for ROS (timestamp added by ROS logging)
    sink->set_formatter(
        expr::stream
            << "[" << expr::attr<std::string>("Component") << "] "
            << expr::smessage
    );
    
    return sink;
}

}  // namespace logging
}  // namespace axon
```

### 6. axon_mcap Logging

Since `axon_mcap` is exclusively used by `axon_recorder`, it uses the **same Boost.Log infrastructure**. No separate logging system needed.

```cpp
// In core/axon_mcap/mcap_writer_wrapper.cpp
#define AXON_LOG_COMPONENT "axon_mcap"
#include <axon_recorder/logging/axon_log_macros.hpp>

bool McapWriterWrapper::Impl::open(const std::string& path, const McapWriterOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_open_) {
        last_error_ = "Writer already open";
        AXON_LOG_WARN("Attempted to open already-open writer");
        return false;
    }

    AXON_LOG_DEBUG("Opening MCAP file") << kv("path", path);
    
    // ... existing code ...
    
    auto status = writer_.open(path, mcap_opts);
    if (!status.ok()) {
        last_error_ = "Failed to open MCAP file: " + status.message;
        AXON_LOG_ERROR("Failed to open MCAP file") 
            << kv("path", path) 
            << kv("error", status.message);
        return false;
    }

    AXON_LOG_INFO("MCAP file opened") << kv("path", path);
    is_open_ = true;
    return true;
}

bool McapWriterWrapper::Impl::write(...) {
    // Hot path - DEBUG compiled out in release builds
    AXON_LOG_DEBUG("Writing message") << kv("channel_id", channel_id);
    
    // ... existing code ...
    
    if (!status.ok()) {
        AXON_LOG_ERROR("Failed to write message") << kv("error", status.message);
        return false;
    }
    return true;
}
```

**Why this is simpler**:
- Single logging API across all components
- No callback bridge code
- Full Boost.Log features in axon_mcap (async, structured logging, context)

### 7. Logger Registry (Boost.Log Initialization)

Global logger configuration and initialization using Boost.Log.

```cpp
// axon_log_init.hpp
#pragma once

#include "axon_log_severity.hpp"
#include "axon_console_sink.hpp"
#include "axon_file_sink.hpp"
#include "axon_ros_sink.hpp"
#include "ros_interface.hpp"
#include <boost/log/core.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <vector>

namespace axon {
namespace logging {

/**
 * Logging configuration for axon_recorder.
 */
struct LoggingConfig {
    // Console sink
    bool console_enabled = true;
    bool console_colors = true;
    severity_level console_level = severity_level::info;
    
    // File sink
    bool file_enabled = true;
    FileSinkConfig file_config;  // Uses FileSinkConfig defaults
    severity_level file_level = severity_level::debug;
    
    // ROS sink
    bool ros_enabled = true;
    severity_level ros_level = severity_level::info;
};

/**
 * Initialize Boost.Log with all sinks.
 * Should be called once at application startup.
 * 
 * @param config Logging configuration
 * @param ros_interface ROS interface for ROS sink (can be nullptr, add later)
 */
void init_logging(const LoggingConfig& config, 
                  recorder::RosInterface* ros_interface = nullptr);

/**
 * Add ROS sink after ROS is initialized.
 * Call this after ROS interface is created if not available at init_logging().
 */
void add_ros_sink(recorder::RosInterface* ros_interface,
                  severity_level min_level = severity_level::info);

/**
 * Shutdown logging infrastructure.
 * Stops async sink threads and flushes all pending records.
 */
void shutdown_logging();

/**
 * Get the global logger instance.
 */
logger_type& get_logger();

}  // namespace logging
}  // namespace axon
```

**Implementation**:

```cpp
// axon_log_init.cpp
#include "axon_log_init.hpp"
#include <boost/log/core.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <vector>

namespace axon {
namespace logging {

namespace {
    // Store sink pointers for cleanup
    std::vector<boost::shared_ptr<boost::log::sinks::sink>> g_sinks;
    boost::shared_ptr<async_console_sink_t> g_console_sink;
    boost::shared_ptr<async_file_sink_t> g_file_sink;
    boost::shared_ptr<ros_sink_t> g_ros_sink;
    
    // Global logger instance
    logger_type g_logger;
}

logger_type& get_logger() {
    return g_logger;
}

void init_logging(const LoggingConfig& config, 
                  recorder::RosInterface* ros_interface) {
    auto core = boost::log::core::get();
    
    // Add common attributes (TimeStamp, ThreadID, etc.)
    boost::log::add_common_attributes();
    
    // Console sink
    if (config.console_enabled) {
        g_console_sink = create_console_sink(config.console_level, config.console_colors);
        core->add_sink(g_console_sink);
        g_sinks.push_back(g_console_sink);
    }
    
    // File sink
    if (config.file_enabled) {
        g_file_sink = create_file_sink(config.file_config, config.file_level);
        core->add_sink(g_file_sink);
        g_sinks.push_back(g_file_sink);
    }
    
    // ROS sink (if interface available)
    if (config.ros_enabled && ros_interface) {
        add_ros_sink(ros_interface, config.ros_level);
    }
}

void add_ros_sink(recorder::RosInterface* ros_interface,
                  severity_level min_level) {
    if (!ros_interface) return;
    
    auto core = boost::log::core::get();
    g_ros_sink = create_ros_sink(ros_interface, min_level);
    core->add_sink(g_ros_sink);
    g_sinks.push_back(g_ros_sink);
}

void shutdown_logging() {
    auto core = boost::log::core::get();
    
    // Stop async sinks first
    if (g_console_sink) {
        g_console_sink->stop();
        g_console_sink->flush();
    }
    if (g_file_sink) {
        g_file_sink->stop();
        g_file_sink->flush();
    }
    
    // Remove all sinks
    for (auto& sink : g_sinks) {
        core->remove_sink(sink);
    }
    g_sinks.clear();
    
    g_console_sink.reset();
    g_file_sink.reset();
    g_ros_sink.reset();
}

}  // namespace logging
}  // namespace axon
```


## Integration with axon_recorder

### Recorder Node Integration

```cpp
// In main.cpp
#define AXON_LOG_COMPONENT "main"
#include "axon_log_init.hpp"
#include "axon_log_macros.hpp"
#include "axon_mcap_log_bridge.hpp"

int main(int argc, char** argv) {
    // Initialize logging before anything else (without ROS sink)
    axon::logging::LoggingConfig log_config;
    log_config.file_config.directory = "/var/log/axon";
    log_config.file_config.format_json = true;
    log_config.file_level = axon::logging::severity_level::debug;
    
    axon::logging::init_logging(log_config, nullptr);
    
    AXON_LOG_INFO("axon_recorder starting") << kv("version", AXON_RECORDER_VERSION);
    
    auto node = axon::recorder::RecorderNode::create();
    if (!node->initialize(argc, argv)) {
        AXON_LOG_FATAL("Initialization failed");
        axon::logging::shutdown_logging();
        return 1;
    }
    
    // Now add ROS sink with initialized interface
    axon::logging::add_ros_sink(
        node->get_ros_interface(), 
        axon::logging::severity_level::info);
    
    AXON_LOG_INFO("Initialization complete, starting main loop");
    
    node->run();
    
    AXON_LOG_INFO("Shutting down");
    node->shutdown();
    
    axon::logging::shutdown_logging();
    return 0;
}
```

### Recording Session Context

```cpp
// In recorder_node.cpp
#define AXON_LOG_COMPONENT "recorder_node"
#include "axon_log_macros.hpp"

bool RecorderNode::start_recording() {
    // Set scoped logging context for this recording session
    // (automatically cleared when scope exits)
    AXON_LOG_SCOPED_CONTEXT(
        task_config_cache_.get().task_id,
        config_.device.id);
    
    AXON_LOG_INFO("Starting recording")
        << kv("scene", task_config_cache_.get().scene)
        << kv("topics", topic_configs_.size());
    
    // ... existing start_recording code ...
}

void RecorderNode::stop_recording() {
    AXON_LOG_INFO("Stopping recording")
        << kv("duration_sec", get_recording_duration_sec())
        << kv("messages", get_stats().messages_written);
    
    // ... existing stop_recording code ...
    // Context automatically cleared when scoped attribute goes out of scope
}
```

### Migration Path

Replace existing logging calls incrementally:

```cpp
// Before:
std::cerr << "[axon_recorder] Entering spin()..." << std::endl;

// After:
AXON_LOG_INFO("Entering spin loop");

// Before:
std::cerr << "[RecordingSession] Failed to open: " << error << std::endl;

// After:
AXON_LOG_ERROR("Failed to open recording session") << kv("error", error);

// Before:
RCLCPP_INFO(get_logger(), "Recording started for task: %s", task_id.c_str());

// After:
AXON_LOG_INFO("Recording started") << kv("task_id", task_id);

// Before (axon_mcap - error stored in string only):
last_error_ = "Failed to write: " + error;

// After (axon_mcap - same AXON_LOG_* macros):
AXON_LOG_ERROR("Failed to write") << kv("error", error);
```

## Configuration

### Integrated YAML Configuration

Logging configuration is integrated into the recorder's main config file (`middlewares/axon_recorder/config/default_config.yaml`):

```yaml
# Axon Recorder Configuration
# This configuration includes recorder settings AND logging configuration

dataset:
  path: /data/recordings
  mode: append

topics:
  - name: /imu/data
    message_type: sensor_msgs/Imu
    batch_size: 5000
    flush_interval_ms: 5000
  # ... more topics ...

recording:
  max_disk_usage_gb: 100

# Logging configuration (parsed by ConfigParser, applied at startup)
# Environment variables can override these settings (see below)
logging:
  console:
    enabled: true
    level: info      # debug, info, warn, error, fatal
    colors: true
  file:
    enabled: false   # Set to true to enable file logging
    level: debug
    directory: /var/log/axon
    pattern: "axon_%Y%m%d_%H%M%S.log"
    format: json     # json or text
    rotation_size_mb: 100
    max_files: 10
    rotate_at_midnight: true
```

### Environment Variable Overrides

Environment variables take precedence over YAML configuration. This allows runtime customization without modifying config files (useful for Docker/K8s deployments):

| Variable | Description | Example |
|----------|-------------|---------|
| `AXON_LOG_LEVEL` | Global level (overrides both console and file) | `debug` |
| `AXON_LOG_CONSOLE_LEVEL` | Console sink level | `info` |
| `AXON_LOG_FILE_LEVEL` | File sink level | `debug` |
| `AXON_LOG_FILE_DIR` | Log file directory | `/var/log/axon` |
| `AXON_LOG_FORMAT` | File format | `json` or `text` |
| `AXON_LOG_FILE_ENABLED` | Enable file logging | `true` or `false` |
| `AXON_LOG_CONSOLE_ENABLED` | Enable console logging | `true` or `false` |

**Priority Order** (highest to lowest):
1. Environment variables
2. YAML config file settings
3. Hardcoded defaults

```bash
# Example: Enable debug logging with file output for troubleshooting
export AXON_LOG_LEVEL=debug
export AXON_LOG_FILE_ENABLED=true
export AXON_LOG_FILE_DIR=/tmp/axon_debug_logs

# Example: Production deployment with JSON file logging
export AXON_LOG_CONSOLE_LEVEL=warn
export AXON_LOG_FILE_ENABLED=true
export AXON_LOG_FORMAT=json
```

### Configuration Loading Flow

```
┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────────┐
│  default_config.yaml│────►│    ConfigParser     │────►│   LoggingConfig     │
│  (logging section)  │     │  parse_logging()    │     │    (struct)         │
└─────────────────────┘     └─────────────────────┘     └──────────┬──────────┘
                                                                   │
                                                                   ▼
┌─────────────────────┐     ┌─────────────────────┐     ┌─────────────────────┐
│  Environment Vars   │────►│ apply_env_overrides │────►│  Final LoggingConfig│
│  (AXON_LOG_*)       │     │       ()            │     │                     │
└─────────────────────┘     └─────────────────────┘     └──────────┬──────────┘
                                                                   │
                                                                   ▼
                                                        ┌─────────────────────┐
                                                        │ reconfigure_logging │
                                                        │       ()            │
                                                        └─────────────────────┘
```

## Performance Considerations

### Hot Path Protection

The recording hot path (message callback → queue → worker → MCAP write) must not be impacted by logging.

**Boost.Log Strategy**:
1. **Async sinks with bounded queues** - Log calls return immediately, I/O in background thread
2. **Drop on overflow** - Under extreme load, logs are dropped rather than blocking
3. **DEBUG compiled out** - `AXON_LOG_ENABLE_DEBUG` is `false` in release builds
4. **Level filtering at source** - Boost.Log filters before record creation
5. **Log sampling macros** - Reduce log volume for high-frequency events

```cpp
// Hot path - async sink + compile-time DEBUG elimination
void WorkerThread::process_message(const MessageItem& item) {
    // In release: expands to nothing
    // In debug: async enqueue, returns immediately
    AXON_LOG_DEBUG("Processing message"
        << kv("topic", topic_)
        << kv("size", item.data.size()));
    
    // ... actual processing (not blocked by logging) ...
}

// High-frequency loops - use count-based sampling
for (const auto& msg : messages) {
    // Log only every 1000th message
    AXON_LOG_DEBUG_EVERY_N(1000, "Processed message" << kv("seq", msg.seq));
}

// Error conditions that may repeat - use time-based throttling
if (buffer_full) {
    // Log at most once per 5 seconds
    AXON_LOG_WARN_THROTTLE(5.0, "Buffer full, dropping messages");
}
```

### Log Sampling Macros

For high-frequency logging scenarios, use the sampling macros to reduce log volume:

| Macro | Pattern | Use Case |
|-------|---------|----------|
| `AXON_LOG_*_EVERY_N(n, msg)` | Log 1st, then every Nth | High-frequency loops, progress updates |
| `AXON_LOG_*_THROTTLE(sec, msg)` | At most once per N seconds | Repeating errors, rate-limited warnings |

```cpp
// Count-based: Logs at count 1, 101, 201, 301, ...
AXON_LOG_INFO_EVERY_N(100, "Processing batch" << kv("count", i));

// Time-based: Logs at most once per 10 seconds regardless of call frequency
AXON_LOG_ERROR_THROTTLE(10.0, "Disk space low" << kv("free_mb", free));
```

### Async Sink Architecture

```
┌─────────────────┐     ┌─────────────────────────────┐     ┌──────────────┐
│ AXON_LOG_INFO() │────►│ bounded_fifo_queue (1000)   │────►│ File I/O     │
│ (returns ~1µs)  │     │ drop_on_overflow if full    │     │ (background) │
└─────────────────┘     └─────────────────────────────┘     └──────────────┘
                                    ▲
                                    │
                        Background thread drains queue
```

### Benchmarking

| Scenario | std::cerr (sync) | Boost.Log (async) | Improvement |
|----------|------------------|-------------------|-------------|
| Console INFO | ~2µs (blocks) | ~500ns (enqueue) | **4x faster** |
| File JSON | ~10µs (blocks) | ~500ns (enqueue) | **20x faster** |
| DEBUG (release) | N/A | 0µs | Compiled out |
| Level filtered | N/A | <50ns | Early exit |
| Queue full | N/A | ~100ns (drop) | No blocking |

**Key**: Logging never blocks the hot path. Worst case is a dropped log record (counter incremented for observability).

## Testing Strategy

### Unit Tests for axon_mcap Logging

Since `axon_mcap` uses the same Boost.Log macros as `axon_recorder`, the tests are identical to the Boost.Log sink tests below. The only difference is the component name (`axon_mcap` vs `recorder_node`).

```cpp
// test_mcap_writer_logging.cpp - Tests McapWriterWrapper logging output

#define AXON_LOG_COMPONENT "axon_mcap"
#include "axon_log_macros.hpp"

TEST(McapWriterLoggingTest, LogsOnOpen) {
    auto backend = boost::make_shared<TestSink>();
    auto sink = boost::make_shared<boost::log::sinks::synchronous_sink<TestSink>>(backend);
    boost::log::core::get()->add_sink(sink);
    
    McapWriterWrapper writer;
    writer.open("/tmp/test.mcap", McapWriterOptions{});
    
    ASSERT_GE(backend->records().size(), 1);
    EXPECT_THAT(backend->records().back(), HasSubstr("MCAP file opened"));
    
    boost::log::core::get()->remove_sink(sink);
}

TEST(McapWriterLoggingTest, LogsOnError) {
    auto backend = boost::make_shared<TestSink>();
    auto sink = boost::make_shared<boost::log::sinks::synchronous_sink<TestSink>>(backend);
    sink->set_filter(severity >= severity_level::error);
    boost::log::core::get()->add_sink(sink);
    
    McapWriterWrapper writer;
    // Try to write without opening - should log error
    writer.write(1, nullptr, 0, 0);
    
    ASSERT_GE(backend->records().size(), 1);
    EXPECT_EQ(backend->severities().back(), severity_level::error);
    
    boost::log::core::get()->remove_sink(sink);
}
```

### Unit Tests for Boost.Log Sinks

```cpp
// test_boost_log_sinks.cpp - Requires Boost.Log

// Test sink for capturing log records
class TestSink : public boost::log::sinks::basic_formatted_sink_backend<char> {
public:
    void consume(boost::log::record_view const& rec, string_type const& formatted) {
        records_.push_back(formatted);
        severities_.push_back(*rec[severity]);
    }
    
    const std::vector<std::string>& records() const { return records_; }
    const std::vector<severity_level>& severities() const { return severities_; }
    void clear() { records_.clear(); severities_.clear(); }
    
private:
    std::vector<std::string> records_;
    std::vector<severity_level> severities_;
};

TEST(BoostLogTest, SeverityFiltering) {
    auto backend = boost::make_shared<TestSink>();
    auto sink = boost::make_shared<boost::log::sinks::synchronous_sink<TestSink>>(backend);
    sink->set_filter(severity >= severity_level::warn);
    
    boost::log::core::get()->add_sink(sink);
    
    AXON_LOG_DEBUG("debug");  // Filtered
    AXON_LOG_INFO("info");    // Filtered
    AXON_LOG_WARN("warn");    // Passed
    AXON_LOG_ERROR("error");  // Passed
    
    EXPECT_EQ(backend->records().size(), 2);
    EXPECT_EQ(backend->severities()[0], severity_level::warn);
    EXPECT_EQ(backend->severities()[1], severity_level::error);
    
    boost::log::core::get()->remove_sink(sink);
}

TEST(BoostLogTest, ScopedContext) {
    auto backend = boost::make_shared<TestSink>();
    auto sink = boost::make_shared<boost::log::sinks::synchronous_sink<TestSink>>(backend);
    
    // Set formatter to capture TaskID
    sink->set_formatter([](auto const& rec, auto& strm) {
        if (auto tid = rec[task_id]) {
            strm << "task=" << *tid << " ";
        }
        strm << rec[boost::log::expressions::smessage];
    });
    
    boost::log::core::get()->add_sink(sink);
    
    {
        AXON_LOG_SCOPED_CONTEXT("task_001", "device_001");
        AXON_LOG_INFO("message with context");
    }
    
    AXON_LOG_INFO("message without context");
    
    EXPECT_THAT(backend->records()[0], HasSubstr("task=task_001"));
    EXPECT_THAT(backend->records()[1], Not(HasSubstr("task=")));
    
    boost::log::core::get()->remove_sink(sink);
}

TEST(BoostLogTest, AsyncSinkDoesNotBlock) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // Create async sink with slow backend
    auto backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
    auto slow_stream = std::make_shared<SlowOutputStream>(5ms);  // 5ms per write
    backend->add_stream(slow_stream);
    
    auto sink = create_console_sink(severity_level::debug, false);
    boost::log::core::get()->add_sink(sink);
    
    // Log 100 messages
    for (int i = 0; i < 100; i++) {
        AXON_LOG_INFO("Message") << kv("i", i);
    }
    
    auto duration = std::chrono::high_resolution_clock::now() - start;
    
    // Should complete much faster than 100 * 5ms = 500ms
    // (async means we don't wait for writes)
    EXPECT_LT(duration, std::chrono::milliseconds(50));
    
    sink->stop();
    boost::log::core::get()->remove_sink(sink);
}

TEST(FileSinkTest, TimeBasedRotation) {
    FileSinkConfig config;
    config.directory = "/tmp/axon_rotation_test";
    config.rotation_size_mb = 1000;  // Large, won't trigger size rotation
    config.rotate_at_midnight = true;
    config.max_files = 5;
    
    auto sink = create_file_sink(config, severity_level::debug);
    boost::log::core::get()->add_sink(sink);
    
    AXON_LOG_INFO("Before rotation");
    
    sink->stop();
    sink->flush();
    
    // Verify file created with timestamp pattern
    auto files = list_files_matching("/tmp/axon_rotation_test/recorder_*.log");
    EXPECT_GE(files.size(), 1);
    
    boost::log::core::get()->remove_sink(sink);
}
```

### Integration Tests

```cpp
// test_logging_integration.cpp

TEST(LoggingIntegrationTest, EndToEndWithBoostLog) {
    LoggingConfig config;
    config.console_enabled = false;
    config.file_enabled = true;
    config.file_config.directory = "/tmp/axon_integration_test";
    config.file_config.format_json = true;
    config.ros_enabled = false;
    
    init_logging(config, nullptr);
    
    {
        AXON_LOG_SCOPED_CONTEXT("test_task_123", "robot_001");
        
        // Log from axon_recorder
        AXON_LOG_INFO("Recording started") << kv("scene", "park");
        
        // Log from axon_mcap (same API, same Boost.Log)
        AXON_LOG_INFO("MCAP file opened");
        AXON_LOG_ERROR("Write failed");
    }
    
    shutdown_logging();
    
    // Read and verify JSON output
    auto files = list_files_matching("/tmp/axon_integration_test/recorder_*.log");
    ASSERT_FALSE(files.empty());
    
    std::ifstream file(files[0]);
    std::vector<nlohmann::json> records;
    std::string line;
    while (std::getline(file, line)) {
        records.push_back(nlohmann::json::parse(line));
    }
    
    ASSERT_GE(records.size(), 3);
    EXPECT_EQ(records[0]["task_id"], "test_task_123");
    EXPECT_EQ(records[0]["device_id"], "robot_001");
}

TEST(LoggingIntegrationTest, GracefulShutdownDrainsQueue) {
    LoggingConfig config;
    config.file_enabled = true;
    config.file_config.directory = "/tmp/axon_shutdown_test";
    
    init_logging(config, nullptr);
    
    // Flood logs
    for (int i = 0; i < 500; i++) {
        AXON_LOG_INFO("Log message") << kv("i", i);
    }
    
    // Shutdown should drain async queue
    shutdown_logging();
    
    // All logs should be written (not dropped during shutdown)
    auto files = list_files_matching("/tmp/axon_shutdown_test/recorder_*.log");
    ASSERT_FALSE(files.empty());
    
    int line_count = count_lines(files[0]);
    EXPECT_EQ(line_count, 500);
}

TEST(LoggingIntegrationTest, McapLogsWithContext) {
    auto backend = boost::make_shared<TestSink>();
    auto sink = boost::make_shared<boost::log::sinks::synchronous_sink<TestSink>>(backend);
    boost::log::core::get()->add_sink(sink);
    
    // axon_mcap uses same logging infrastructure, so context is inherited
    {
        AXON_LOG_SCOPED_CONTEXT("test_task", "robot_001");
        
        // Simulate McapWriterWrapper logging
        AXON_LOG_ERROR("MCAP write failure") << kv("channel_id", 5);
    }
    
    ASSERT_EQ(backend->records().size(), 1);
    EXPECT_THAT(backend->records()[0], HasSubstr("MCAP write failure"));
    EXPECT_EQ(backend->severities()[0], severity_level::error);
    
    boost::log::core::get()->remove_sink(sink);
}
```

## Build Integration

### Dependencies

```bash
# Ubuntu/Debian - Install Boost.Log and dependencies
sudo apt-get install libboost-log-dev libboost-thread-dev libboost-filesystem-dev

# Or install full Boost (recommended for consistency)
sudo apt-get install libboost-all-dev
```

### CMakeLists.txt for axon_mcap

Since `axon_mcap` is only used by `axon_recorder`, it links to the shared `axon_logging` library.

```cmake
# In core/axon_mcap/CMakeLists.txt

# axon_mcap uses the same logging as axon_recorder
target_link_libraries(axon_mcap PUBLIC
    mcap::mcap
)

# Note: axon_logging is linked at the axon_recorder level since axon_mcap
# is compiled as part of axon_recorder. The logging macros are header-only.
```

### CMakeLists.txt for axon_recorder (Boost.Log)

```cmake
# In middlewares/axon_recorder/CMakeLists.txt - Boost.Log integration

# Find Boost.Log and dependencies
find_package(Boost 1.74 REQUIRED COMPONENTS 
    log 
    log_setup 
    thread 
    filesystem
)

# Logging library (Boost.Log + ROS integration)
add_library(axon_logging
    src/logging/axon_log_init.cpp
    src/logging/axon_console_sink.cpp
    src/logging/axon_file_sink.cpp
    src/logging/axon_ros_sink.cpp
)

target_include_directories(axon_logging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Define BOOST_LOG_DYN_LINK if using shared Boost libraries
target_compile_definitions(axon_logging PRIVATE
    BOOST_LOG_DYN_LINK
)

target_link_libraries(axon_logging PUBLIC
    Boost::log
    Boost::log_setup
    Boost::thread
    Boost::filesystem
)

# Link logging to recorder
target_link_libraries(axon_recorder_lib
    axon_logging
    axon_mcap  # Includes callback-based mcap_logging (no Boost)
)
```

### File Structure

```
core/
├── axon_logging/                       # Standalone logging library (no ROS deps)
│   ├── CMakeLists.txt
│   ├── axon_log_severity.hpp           # Severity enum + keywords
│   ├── axon_log_macros.hpp             # AXON_LOG_* macros + sampling macros
│   ├── axon_console_sink.hpp           # Async console sink
│   ├── axon_console_sink.cpp
│   ├── axon_file_sink.hpp              # Async file sink + rotation
│   ├── axon_file_sink.cpp
│   ├── axon_log_init.hpp               # Initialization + env var overrides
│   └── axon_log_init.cpp
│
└── axon_mcap/                          # MCAP wrapper (uses axon_logging)
    ├── CMakeLists.txt                  # Links to axon_logging
    ├── mcap_writer_wrapper.hpp
    └── mcap_writer_wrapper.cpp         # Uses AXON_LOG_* macros

middlewares/
└── axon_recorder/
    ├── CMakeLists.txt                  # Links axon_logging + axon_mcap
    ├── config/
    │   └── default_config.yaml         # Includes logging: section
    └── src/
        ├── config_parser.hpp           # Parses logging: section from YAML
        ├── config_parser.cpp           # convert_logging_config() helper
        ├── recorder_node.cpp           # Calls reconfigure_logging() after config load
        └── logging/
            ├── axon_ros_sink.hpp       # Custom ROS backend (ROS-specific)
            └── axon_ros_sink.cpp
```

**Key Points**:
- `core/axon_logging/` is a standalone library with no ROS dependencies
- `core/axon_mcap/` links to `axon_logging` and uses the same macros
- `middlewares/axon_recorder/` links both and adds ROS-specific sink
- Logging config is integrated into `default_config.yaml`, not a separate file

### Docker Integration

```dockerfile
# In Dockerfile - Install Boost.Log
RUN apt-get update && apt-get install -y \
    libboost-log-dev \
    libboost-thread-dev \
    libboost-filesystem-dev \
    && rm -rf /var/lib/apt/lists/*

# Create log directory
RUN mkdir -p /var/log/axon && chmod 755 /var/log/axon
```

## Deployment Considerations

### Log Directory Setup

```bash
# Create log directory with appropriate permissions
sudo mkdir -p /var/log/axon
sudo chown axon:axon /var/log/axon

# Or use /data/logs for edge devices
mkdir -p /data/logs/axon
```

### Logrotate Configuration (Optional)

For external rotation management:

```
# /etc/logrotate.d/axon
/var/log/axon/*.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 0644 axon axon
}
```

**Note**: If using the built-in file rotation, external logrotate is not needed.

### Docker Integration

```dockerfile
# In Dockerfile
RUN mkdir -p /var/log/axon

# Mount log volume in docker-compose
volumes:
  - ./logs:/var/log/axon
```

## Future Enhancements

| Enhancement | Priority | Status | Description |
|-------------|----------|--------|-------------|
| **Async file sink** | Medium | ✅ Done | Background thread for file writes |
| **Log sampling** | High | ✅ Done | Count-based and time-based sampling macros |
| **Config file support** | High | ✅ Done | YAML configuration integrated into recorder config |
| **Environment variables** | High | ✅ Done | Runtime overrides via `AXON_LOG_*` env vars |
| **Runtime level change** | Low | Planned | ROS service to change log level dynamically |
| **Log aggregation** | Low | Planned | Integration with Fluentd/Logstash |
| **Metrics emission** | Medium | Planned | Export log counts to Prometheus |
| **Structured context** | Medium | Planned | Typed context fields for JSON |
| **Remote log shipping** | Low | Planned | Syslog, cloud backend integration |

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-22 | - | Initial design |
| 1.1 | 2025-12-22 | - | Added `core/axon_mcap/` logging support with callback-based approach for ROS-independent operation |
| 1.2 | 2025-12-22 | - | Replaced custom logging with Boost.Log for all components. Since `axon_mcap` is only a wrapper for Foxglove MCAP and exclusively used by `axon_recorder`, it uses the same Boost.Log infrastructure (no separate callback-based system needed). |
| 1.3 | 2025-12-22 | - | Added log sampling macros (`AXON_LOG_*_EVERY_N`, `AXON_LOG_*_THROTTLE`) for high-frequency events. Integrated logging config into recorder's `default_config.yaml`. Added environment variable overrides with priority system. Reorganized file structure with standalone `core/axon_logging/` library. |


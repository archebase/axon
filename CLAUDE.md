# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Axon** is a high-performance ROS (Robot Operating System) recorder by ArcheBase that writes sensor data to MCAP format. It supports both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling) distributions with a task-centric design for fleet management scenarios where a server controls recording via ros-bridge.

**Key Design Philosophy:**
- Task-centric: One task = one MCAP file with full lifecycle management
- Lock-free: Per-topic SPSC queues for zero-copy message handling
- Fleet-ready: Server-controlled recording via HTTP callbacks
- Crash-resilient: S3 uploader with SQLite state persistence and recovery

## Build and Test Commands

**All commands should be run from the `ros/` directory unless specified otherwise.**

### Primary Development Commands

```bash
# From ros/ directory
cd ros

# Build everything (auto-detects ROS version)
make build

# Run all tests
make test

# Clean all build artifacts
make clean

# Show all available targets
make help
```

### Component-Specific Commands

```bash
# Build only C++ code
make cpp-build

# Build only Rust bridge (C FFI)
make rust-build

# Run only C++ tests
make cpp-test

# Run only Rust tests
make rust-test
```

### Build Modes

```bash
# Debug build
make debug

# Release build (default)
make release
```

### Docker Testing (No Local ROS Required)

```bash
# Test specific ROS version
make docker-test-ros1              # ROS1 Noetic
make docker-test-ros2-humble      # ROS2 Humble
make docker-test-ros2-jazzy       # ROS2 Jazzy
make docker-test-ros2-rolling     # ROS2 Rolling

# Test all versions sequentially
make docker-test-all

# Test all versions in parallel (faster)
make docker-test-compose

# Coverage in Docker
make docker-coverage
```

### Code Quality

```bash
# Format code (requires clang-format and cargo)
make format

# Lint code (requires cppcheck and clippy)
make lint
```

### Coverage Reports

```bash
# Generate coverage report (requires lcov, ROS2 Humble recommended)
make coverage

# Generate HTML coverage report
make coverage-html

# Clean coverage data
make clean-coverage
```

## High-Level Architecture

The system follows a layered architecture with a 4-state task-centric FSM:

```
Server/Fleet Manager (ros-bridge) → Recording Services → State Machine → MCAP Writer
                                     ↓                    ↓            ↓
                              HTTP Callbacks       Worker Threads   SPSC Queues
                                     ↓                    ↓            ↓
                           CachedRecordingConfig   Start/Finish   Lock-free
                              (YAML Configuration)   Notify      Message Transfer
```

### State Machine

```
IDLE → READY → RECORDING ↔ PAUSED
  ↑                         ↓
  └───────── (finish/cancel)
```

**States:**
- `IDLE`: No active task, waiting for configuration
- `READY`: Task config cached, ready to start recording
- `RECORDING`: Actively recording ROS messages to MCAP
- `PAUSED`: Recording paused, can resume or finish

### Core Components

**1. ROS Recorder Node** ([ros/src/axon_recorder/](ros/src/axon_recorder/))
   - Service-based API for server integration
   - State machine with thread-safe transitions
   - Worker thread pool with per-topic SPSC queues
   - HTTP callback client for server notifications
   - Metadata injector (MCAP metadata + sidecar JSON)

**2. MCAP Writer Library** ([cpp/axon_mcap/](cpp/axon_mcap/))
   - Thread-safe MCAP file operations
   - Zstd/LZ4 compression support
   - Schema/channel registration
   - File integrity validation

**3. Edge Uploader** ([cpp/axon_uploader/](cpp/axon_uploader/))
   - S3 multipart upload for large files
   - SQLite state persistence for crash recovery
   - Exponential backoff retry with jitter
   - MCAP-first, JSON-last upload order

**4. Logging Infrastructure** ([cpp/axon_logging/](cpp/axon_logging/))
   - Boost.Log with async sinks
   - Console, file, and ROS sinks
   - Severity filtering and rotation

### Threading Model

| Thread | Responsibility |
|--------|----------------|
| ROS Executor | Service callbacks, subscription callbacks |
| Worker Threads | Drain SPSC queues, write to MCAP (one per topic) |
| HTTP Client | Async callbacks to server |
| Uploader Workers | S3 upload with retry |

## Critical Rules and Conventions

### ROS1 and ROS2 Dual Support

**This codebase MUST support both ROS1 and ROS2.** All changes to ROS code must maintain compatibility.

**Conditional Compilation:**
```cpp
#if defined(AXON_ROS1)
  // ROS1-specific code
  #include <ros/ros.h>
#elif defined(AXON_ROS2)
  // ROS2-specific code
  #include <rclcpp/rclcpp.hpp>
#endif
```

**Modern CMake Targets (CRITICAL for ROS2):**
- **DO NOT** use `ament_target_dependencies()` - deprecated in ROS2 Rolling
- **USE** modern `target_link_libraries()` with explicit targets:
  ```cmake
  # GOOD (works on Humble/Jazzy/Rolling):
  target_link_libraries(my_target
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
  )

  # BAD (deprecated, causes warnings in Rolling):
  ament_target_dependencies(my_target rclcpp std_msgs)
  ```

### Commit Message Convention

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
type(scope): description

Types: feat, fix, docs, style, refactor, perf, test, build, ci, chore

Scopes: recorder, mcap, uploader, logging, config, test

Examples:
feat(recorder): add support for new sensor type
fix(mcap): resolve file corruption on crash
test(uploader): improve retry logic coverage
```

Keep descriptions under 72 characters. Use imperative mood ("add" not "added").

### No Unsolicited Documentation

**NEVER create documentation files without explicit user request.**

- Prohibited: README.md, SUMMARY.md, CHANGELOG.md, FIXES_SUMMARY.md, design docs
- Unless the user explicitly asks: "create a doc", "write documentation", etc.
- Focus on code changes; explain what you did in chat response

### Common ROS1 vs ROS2 Patterns

| Feature | ROS1 | ROS2 |
|---------|------|------|
| Node | `ros::NodeHandle` | `rclcpp::Node` |
| Publisher | `ros::Publisher` | `rclcpp::Publisher<T>` |
| Subscriber | `ros::Subscriber` | `rclcpp::Subscription<T>` |
| Service | `ros::ServiceServer` | `rclcpp::Service<T>` |
| Timer | `ros::Timer` | `rclcpp::TimerBase` |
| Logging | `ROS_INFO()` | `RCLCPP_INFO()` |

## Testing Strategy

- **Unit Tests**: Core library components ([cpp/axon_mcap/test/](cpp/axon_mcap/test/), [cpp/axon_logging/test/](cpp/axon_logging/test/), etc.)
- **Integration Tests**: ROS service API, state machine transitions ([ros/src/axon_recorder/test/integration/](ros/src/axon_recorder/test/integration/))
- **E2E Tests**: Full recording workflow with Docker ([ros/src/axon_recorder/test/e2e/](ros/src/axon_recorder/test/e2e/))

**Running Tests:**
```bash
# Local (requires ROS environment)
make test

# Docker (no ROS required, recommended for CI)
make docker-test
```

## Pull Request Description Template

When generating PR descriptions, use the following structure:

```markdown
## Summary

[Concise 1-2 sentence description]

## Motivation

[Why this change is needed]

## Changes

[Detailed technical description with file links]

## Testing

- [ ] Test case 1
- [ ] Test case 2
- [ ] Additional test scenarios

## Backward Compatibility

[Description of backward compatibility impact]

## Related Files

**Modified:**
- [path/to/file1.ext](path/to/file1.ext)
- [path/to/file2.ext](path/to/file2.ext)

**Added:**
- [path/to/new_file.ext](path/to/new_file.ext)

**Deleted:**
- [path/to/old_file.ext](path/to/old_file.ext)
```

**Guidelines:**
- Use markdown link syntax for file references: `[filename](path/to/file)`
- Include specific line numbers when referencing code: `[filename](path/to/file#L42)`
- Use present tense ("Changes X to Y" not "Changed X to Y")
- Be honest about testing limitations

## Configuration

- YAML-based configuration files for recording tasks
- Environment variable support for all settings
- Cascading config resolution: env → config → ROS params
- Default configurations in [ros/src/axon_recorder/config/](ros/src/axon_recorder/config/)

## Key Technologies

- **ROS**: ROS 1 (Noetic) and ROS 2 (Humble/Jazzy/Rolling)
- **Build Systems**: CMake for C++, catkin for ROS1, colcon for ROS2
- **Threading**: Lock-free SPSC queues, worker thread pools
- **Message Format**: MCAP (append-only container, Foxglove compatible)
- **Compression**: Zstd, LZ4
- **Networking**: HTTP/HTTPS callbacks, JWT authentication
- **Storage**: SQLite for state persistence, S3 for cloud storage

## Important References

- [README.md](README.md) - User-facing documentation
- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed system architecture
- [docs/](docs/) - Design documents for individual components
- [.cursor/rules/](.cursor/rules/) - Development rules (conventional commits, ROS dual support, no unsolicited docs)

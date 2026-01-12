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

**All commands should be run from the project root directory.**

### Primary Development Commands

```bash
# From project root

# Build everything (auto-detects ROS version)
make build

# Run all tests
make test

# Clean all build artifacts
make clean

# Show all available targets
make help
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

The system follows a layered architecture with a 4-state task-centric FSM and a plugin-based middleware integration layer:

```
Server/Fleet Manager (ros-bridge) → Recording Services → State Machine → MCAP Writer
                                     ↓                    ↓            ↓
                              HTTP Callbacks       Worker Threads   SPSC Queues
                                     ↓                    ↓            ↓
                           CachedRecordingConfig   Start/Finish   Lock-free
                              (YAML Configuration)   Notify      Message Transfer
```

### Plugin-Based Middleware Architecture

Axon uses a **plugin-based middleware integration** that cleanly separates middleware-specific code from core functionality:

**Key Design Principles:**
- **Middleware Isolation**: Each middleware (ROS1, ROS2) resides in `middlewares/{name}/` and compiles independently into a dynamic library
- **Core Independence**: All components outside `middlewares/` are middleware-agnostic with no ROS dependencies
- **Unified C API**: Middlewares expose a unified C API interface for integration with the main application
- **Dynamic Plugin Loading**: The main program loads middleware plugins as dynamic libraries at runtime via `dlopen/dlsym`

**Directory Structure:**
```
Axon/
├── core/                      # Middleware-agnostic core libraries
│   ├── axon_mcap/            # MCAP writer (no ROS dependencies)
│   ├── axon_logging/         # Logging infrastructure (no ROS dependencies)
│   └── axon_uploader/        # S3 uploader (no ROS dependencies)
│
├── middlewares/              # Middleware-specific plugins
│   ├── ros1/                 # ROS1 (Noetic) plugin → libaxon_ros1.so
│   └── ros2/                 # ROS2 (Humble/Jazzy/Rolling) plugin → libaxon_ros2.so
│
├── apps/                     # Main applications
│   └── axon_recorder/        # Plugin loader and main application
│
└── include/                  # Unified plugin interface
    └── middleware_abi.h      # C API definitions for plugin communication
```

**Benefits:**
1. Core libraries have zero middleware dependencies
2. New middlewares can be added without touching core code
3. Core libraries can be tested independently of ROS
4. Only required middleware plugins need to be deployed
5. Middleware-specific bugs are isolated to plugin code

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

**1. ROS Recorder Node** ([middlewares/src/axon_recorder/](middlewares/src/axon_recorder/))
   - Service-based API for server integration
   - State machine with thread-safe transitions
   - Worker thread pool with per-topic SPSC queues
   - HTTP callback client for server notifications
   - Metadata injector (MCAP metadata + sidecar JSON)

**2. MCAP Writer Library** ([core/axon_mcap/](core/axon_mcap/))
   - Thread-safe MCAP file operations
   - Zstd/LZ4 compression support
   - Schema/channel registration
   - File integrity validation

**3. Edge Uploader** ([core/axon_uploader/](core/axon_uploader/))
   - S3 multipart upload for large files
   - SQLite state persistence for crash recovery
   - Exponential backoff retry with jitter
   - MCAP-first, JSON-last upload order

**4. Logging Infrastructure** ([core/axon_logging/](core/axon_logging/))
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

## Running Individual Tests

```bash
# Run all tests
make test

# Run specific library tests
make test-mcap       # MCAP writer tests
make test-uploader   # Edge uploader tests
make test-logging    # Logging infrastructure tests

# Run a single test from the build directory
cd core/build
ctest -R test_mcap_writer -V          # Run with verbose output
ctest -R test_mcap_validator --output-on-failure

# Run tests with specific labels (ROS tests)
cd middlewares/build/axon_recorder
ctest -L integration --output-on-failure  # Run only integration tests
```

## Critical Rules and Conventions

### CMake Test Auto-Discovery

**Use auto-discovery patterns instead of manually listing each test file.** Tests are automatically discovered by pattern matching:

```bash
# Just create a new test file - CMake will find it
touch core/axon_mcap/test/test_my_new_feature.cpp
```

The CMake files use `GLOB CONFIGURE_DEPENDS` to auto-discover `test_*.cpp` files, so new tests are automatically registered without modifying CMakeLists.txt.

### Header Include Requirements

**Always include all necessary standard library headers explicitly.** Do not rely on transitive includes:

```cpp
// Required headers (include what you use)
#include <atomic>      // for std::atomic
#include <memory>      // for std::unique_ptr, std::shared_ptr
#include <mutex>       // for std::mutex, std::lock_guard
#include <string>      // for std::string
#include <vector>      // for std::vector
#include <cstdint>     // for std::uint8_t, std::int64_t
#include <cstring>     // for std::memcpy, std::memset
```

**Include Order Convention:**
1. Main header (for .cpp files)
2. C system headers (`<cstring>`, `<cstdint>`)
3. C++ standard library headers
4. Third-party library headers
5. Project headers

Separate each group with a blank line.

### C++ Best Practices

**Core Principles:**
- **RAII**: Use smart pointers (`std::unique_ptr`, `std::shared_ptr`) instead of raw pointers for automatic resource management
- **Rule of Zero/Three/Five**: If your class doesn't manage resources, don't define special member functions. If it does, define all five
- **Const Correctness**: Mark functions `const` whenever possible, use `constexpr` for compile-time constants
- **Override Keyword**: Always use `override` for virtual function overrides to catch signature mismatches at compile time
- **Noexcept**: Mark functions that don't throw exceptions (especially move constructors)
- **Pass-by-const-reference**: For large objects to avoid unnecessary copies

**Example:**
```cpp
// GOOD: Rule of Zero with smart pointers
class ModernResourceManager {
public:
  ModernResourceManager() : resource_(std::make_unique<Resource>()) {}
  // All special member functions use defaults - smart pointer handles cleanup

private:
  std::unique_ptr<Resource> resource_;
};

// GOOD: Const correctness and override
class Derived : public Base {
public:
  void do_work() override {  // Compiler error if signature doesn't match
    // implementation
  }

  int compute() const {  // Doesn't modify object state
    return cached_value_;
  }

private:
  int cached_value_;
};
```

### Minimal, Cautious Code Changes

**Make the smallest change that solves the problem.** Avoid refactoring unrelated code or adding infrastructure unless explicitly needed:

1. **Investigate root cause first** - Understand why the problem exists before fixing
2. **Minimal scope** - Fix only the specific issue, don't rewrite surrounding code
3. **No over-engineering** - Don't add patterns or abstractions for simple bugs
4. **Test incrementally** - Make one change, verify it works, then proceed

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

**Note**: With the new plugin architecture, most ROS-specific code is isolated within `middlewares/ros1/` and `middlewares/ros2/` plugins. Core code in `core/` and `apps/` should remain middleware-agnostic.

## Refactoring Guidelines

**When refactoring code in this codebase, follow these principles:**

1. **Investigate First**: Never propose refactoring solutions without understanding the full context. Map the architecture, identify dependencies, and understand data flow before making changes.

2. **Consider Robotics/HPC Constraints**:
   - Real-time constraints and latency sensitivity
   - Thread safety and lock contention
   - Memory access patterns and cache locality
   - Throughput requirements (messages/sec, bytes/sec)

3. **Understand Plugin Architecture**: When refactoring middleware code, remember the plugin-based design. Changes should maintain the clean separation between middleware-specific plugins and middleware-agnostic core.

4. **Minimal Scope**: Make the smallest change that solves the problem. Avoid refactoring unrelated code or adding infrastructure unless explicitly needed.

5. **Ask for Guidance**: If unsure where to start or what the motivation is, ask the user to point you to specific files/modules and explain the context.

## Testing Strategy

- **Unit Tests**: Core library components ([core/axon_mcap/test/](core/axon_mcap/test/), [core/axon_logging/test/](core/axon_logging/test/), etc.)
- **Integration Tests**: ROS service API, state machine transitions ([middlewares/src/axon_recorder/test/integration/](middlewares/src/axon_recorder/test/integration/))
- **E2E Tests**: Full recording workflow with Docker ([middlewares/src/axon_recorder/test/e2e/](middlewares/src/axon_recorder/test/e2e/))

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
- Default configurations in [apps/axon_recorder/config/](apps/axon_recorder/config/)

### Task Configuration Structure

The `task_config.hpp/cpp` files define the core task configuration that flows through the system:

**Key Components:**
- `TaskConfig`: Contains task_id, device_id, scene, topics list, callback URLs, authentication tokens
- `CachedRecordingConfig`: Service that caches task configuration (transitions IDLE → READY)
- Configuration is validated and cached before recording starts
- Used throughout: metadata injection, HTTP callbacks, MCAP metadata

**Typical Flow:**
1. Server calls `CachedRecordingConfig` service with task parameters
2. Config is validated and cached in memory
3. `RecordingControl::start` uses cached config to initialize recording
4. Config embedded in MCAP metadata and sidecar JSON

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
- [.cursor/rules/](.cursor/rules/) - Additional development rules (refactoring guidelines, C++ best practices)

## Quick Reference

### Adding a New Feature

1. **Investigate**: Read related code to understand patterns
2. **Plan**: Identify which files need changes (core vs middleware vs apps)
3. **Implement**: Make minimal, focused changes
4. **Test**: Add tests (CMake auto-discovery will find them)
5. **Format**: Run `make format` and `make lint`

### Debugging a Test Failure

```bash
# Run specific test with verbose output
cd core/build
ctest -R test_mcap_writer -V

# Run tests with specific labels
cd middlewares/build/axon_recorder
ctest -L integration --output-on-failure
```

### Checking ROS Version

```bash
# Check which ROS is sourced
echo $ROS_DISTRO

# Verify ROS1 vs ROS2 build flags
grep -r "AXON_ROS1\|AXON_ROS2" build/
```

### Common File Locations

| Purpose | Location |
|---------|----------|
| Core libraries | `core/axon_*/` |
| ROS plugins | `middlewares/ros1/`, `middlewares/ros2/` |
| Main app | `apps/axon_recorder/` |
| Tests | `*/test/` or `*/test_*.cpp` |
| Configs | `apps/axon_recorder/config/` |
| CMake modules | `cmake/` |
| Plugin interface | `include/middleware_abi.h` |

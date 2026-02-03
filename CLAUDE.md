# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Axon** is a high-performance ROS (Robot Operating System) recorder by ArcheBase that writes sensor data to MCAP format. It supports both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling) distributions with a task-centric design for fleet management scenarios where a server controls recording via HTTP RPC API.

**Key Design Philosophy:**
- Task-centric: One task = one MCAP file with full lifecycle management
- Lock-free: Per-topic SPSC queues for zero-copy message handling
- Fleet-ready: Server-controlled recording via HTTP RPC API (see [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md))
- Crash-resilient: S3 uploader with SQLite state persistence and recovery
- Plugin-based: Middleware-agnostic core with ROS1/ROS2/Zenoh plugins
- Filter-ready: Extensible data processing pipeline for compression and transformation

## Build and Test Commands

**All commands should be run from the project root directory.**

### Primary Development Commands

```bash
# From project root

# Build C++ core libraries (no ROS required)
make build-core

# Build all applications
make app

# Build ROS1 middleware
make build-ros1

# Build ROS2 middleware
make build-ros2

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
# C++ tests (dedicated Docker image, faster than ROS images)
make docker-test-cpp              # C++ core library tests

# ROS tests
make docker-test-ros1             # ROS1 Noetic
make docker-test-ros2-humble      # ROS2 Humble
make docker-test-ros2-jazzy       # ROS2 Jazzy
make docker-test-ros2-rolling     # ROS2 Rolling

# Test all ROS versions sequentially
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
├── middlewares/              # Middleware-specific plugins and filters
│   ├── ros1/                 # ROS1 (Noetic) plugin → libaxon_ros1.so
│   ├── ros2/                 # ROS2 (Humble/Jazzy/Rolling) plugin → libaxon_ros2.so
│   │   └── src/ros2_plugin/  # ROS2 plugin implementation
│   ├── zenoh/                # Zenoh plugin → libaxon_zenoh.so
│   └── filters/              # Data processing filters (shared across plugins)
│       └── depthlitez/       # Depth image compression library
│
├── apps/                     # Main applications
│   ├── axon_recorder/        # Plugin loader and HTTP RPC server
│   └── plugin_example/       # Example plugin implementation
│
└── docs/designs/             # Design documents
    ├── rpc-api-design.md     # HTTP RPC API specification
    └── depth-compression-filter.md  # Depth compression design
```

**Plugin ABI Interface:**
The plugin interface is defined in [apps/axon_recorder/plugin_loader.hpp](apps/axon_recorder/plugin_loader.hpp) with the following C structures:
- `AxonPluginDescriptor`: Plugin metadata and ABI version
- `AxonPluginVtable`: Function pointers for init/start/stop/subscribe/publish
- Each plugin exports `axon_get_plugin_descriptor()` function

**Benefits:**
1. Core libraries have zero middleware dependencies
2. New middlewares can be added without touching core code
3. Core libraries can be tested independently of ROS
4. Only required middleware plugins need to be deployed
5. Middleware-specific bugs are isolated to plugin code
6. Filters in `middlewares/filters/` can be shared across plugins

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

**1. Axon Recorder** ([apps/axon_recorder/](apps/axon_recorder/))
   - HTTP RPC server for remote control (see [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md))
   - Plugin loader for dynamic middleware integration
   - State machine with thread-safe transitions
   - Worker thread pool with per-topic SPSC queues
   - HTTP callback client for server notifications
   - Metadata injector (MCAP metadata + sidecar JSON)

**Key Files:**
- [apps/axon_recorder/axon_recorder.cpp](apps/axon_recorder/axon_recorder.cpp) - Main entry point
- [apps/axon_recorder/http_server.cpp](apps/axon_recorder/http_server.cpp) - HTTP RPC server implementation
- [apps/axon_recorder/recorder.cpp](apps/axon_recorder/recorder.cpp) - Core recording logic
- [apps/axon_recorder/plugin_loader.cpp](apps/axon_recorder/plugin_loader.cpp) - Plugin loading system
- [apps/axon_recorder/state_machine.cpp](apps/axon_recorder/state_machine.cpp) - State machine implementation

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

**5. Depth Compression Filter** ([middlewares/filters/depthlitez/](middlewares/filters/depthlitez/))
   - Specialized compression for 16-bit depth images
   - 5-10x compression ratio using DepthLiteZ algorithm
   - Configurable compression levels (fast/medium/max)
   - Integrates with ROS2 plugin via `DepthCompressionFilter`
   - See [docs/designs/depth-compression-filter.md](docs/designs/depth-compression-filter.md)

### Threading Model

| Thread | Responsibility |
|--------|----------------|
| HTTP RPC Server | Handles incoming HTTP requests (config/begin/end/pause/resume/quit/status) |
| Plugin Executor | ROS executor for subscription callbacks (via plugin) |
| Worker Threads | Drain SPSC queues, write to MCAP (one per topic) |
| HTTP Client | Async callbacks to server (start/finish notifications) |
| Uploader Workers | S3 upload with retry |

### HTTP RPC API

The recorder exposes an HTTP RPC server (default port 8080) for remote control:

**Endpoints:**
- `POST /rpc/config` - Set task configuration (IDLE → READY)
- `POST /rpc/begin` - Start recording (READY → RECORDING)
- `POST /rpc/end` - Finish recording (RECORDING/PAUSED → IDLE)
- `POST /rpc/pause` - Pause recording (RECORDING → PAUSED)
- `POST /rpc/resume` - Resume recording (PAUSED → RECORDING)
- `POST /rpc/quit` - Shutdown recorder
- `GET /rpc/status` - Query current status and metrics

See [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) for complete API specification including request/response formats.

## Running Individual Tests

```bash
# Run C++ core library tests
make test-core

# Run specific library tests
make test-mcap       # MCAP writer tests
make test-uploader   # Edge uploader tests
make test-logging    # Logging infrastructure tests

# Run a single test from the build directory
cd core/build
ctest -R test_mcap_writer -V          # Run with verbose output
ctest -R test_mcap_validator --output-on-failure
```

## CI Testing (Mirror CI Locally)

All CI targets run in Docker to match the CI environment exactly. No local ROS required.

```bash
# Quick CI check (format + lint + C++ tests)
make ci

# Fastest check (format + lint only, no tests)
make ci-quick

# Full CI validation (all checks)
make ci-all

# C++ tests
make ci-cpp                # C++ library tests (unit + integration)
make ci-cpp-unit           # C++ unit tests only
make ci-cpp-integration    # C++ integration tests (with MinIO)

# ROS tests
make ci-ros1               # ROS1 Noetic tests in Docker
make ci-ros2               # ROS2 (Humble + Jazzy + Rolling) tests in Docker
make ci-ros                # All ROS tests

# E2E tests
make ci-e2e                # E2E tests (ROS1 + ROS2 Humble)

# Coverage
make ci-coverage           # All coverage reports (C++ + ROS2 + Recorder)
make ci-coverage-cpp       # C++ library coverage
make ci-coverage-ros       # ROS2 coverage (Humble only)
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
- **E2E Tests**: Full recording workflow with Docker ([apps/axon_recorder/test/e2e/](apps/axon_recorder/test/e2e/))

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
- **Build System**: Unified CMake build system for all components
- **Threading**: Lock-free SPSC queues, worker thread pools
- **Message Format**: MCAP (append-only container, Foxglove compatible)
- **Compression**: Zstd, LZ4
- **Networking**: HTTP/HTTPS callbacks, JWT authentication
- **Storage**: SQLite for state persistence, S3 for cloud storage

## Important References

- [README.md](README.md) - User-facing documentation
- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed system architecture
- [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) - HTTP RPC API specification
- [CONTRIBUTING.md](CONTRIBUTING.md) - Development workflow and contribution guidelines

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
| ROS1 plugin | `middlewares/ros1/src/ros1_plugin/` (CMake) |
| ROS2 plugin | `middlewares/ros2/src/ros2_plugin/` (CMake) |
| Zenoh plugin | `middlewares/zenoh/` (CMake) |
| Filters | `middlewares/filters/` (shared data processing) |
| Depth compression | `middlewares/filters/depthlitez/` |
| Main app (HTTP RPC) | `apps/axon_recorder/` |
| Plugin example | `apps/plugin_example/` |
| Plugin ABI interface | `apps/axon_recorder/plugin_loader.hpp` |
| Tests | `*/test/` or `*/test_*.cpp` |
| CMake modules | `cmake/` |
| Design docs | `docs/designs/` |

### Plugin Development

To create a new middleware plugin:

1. **Define the plugin ABI** - Implement the C interface in [apps/axon_recorder/plugin_loader.hpp](apps/axon_recorder/plugin_loader.hpp)
2. **Export descriptor function** - Each plugin must export `axon_get_plugin_descriptor()`
3. **Compile as shared library** - Build as `.so` with C linkage for ABI functions
4. **Example reference** - See [apps/plugin_example/](apps/plugin_example/) and [middlewares/ros2/src/ros2_plugin/](middlewares/ros2/src/ros2_plugin/)

**ABI Versioning:**
- `AxonPluginDescriptor` contains `abi_version_major` and `abi_version_minor`
- Always verify compatibility before loading plugins
- Reserve space in vtable for future extensions

# Project Structure Design

This document describes the directory structure and organization of the Axon project.

## Overview

Axon follows a **layered architecture** with clear separation between:
- **Core libraries** (`core/`) - ROS-agnostic C++ components
- **Middleware** (`middlewares/`) - ROS 1/2 specific integration code
- **Applications** (`apps/`) - Standalone executables (currently empty, reserved for future use)

This design enables:
- Single codebase supporting both ROS 1 (Noetic) and ROS 2 (Humble, Jazzy, Rolling)
- Reusable core libraries that can be independently tested and maintained
- Clear dependency boundaries and minimal coupling

## Directory Tree

```
Axon/
├── core/                          # Core C++ libraries (ROS-agnostic)
│   ├── axon_logging/
│   ├── axon_mcap/
│   └── axon_uploader/
│
├── middlewares/                   # ROS-specific middleware code
│   ├── ros1/                      # ROS 1 (Noetic) packages
│   │   └── src/
│   │       └── ros1_plugin/
│   │           ├── src/
│   │           ├── srv/
│   │           ├── config/
│   │           ├── launch/
│   │           └── test/
│   └── ros2/                      # ROS 2 (Humble/Jazzy/Rolling) packages
│       └── src/
│           └── ros1_plugin/
│               ├── src/
│               ├── srv/
│               ├── config/
│               ├── launch/
│               └── test/
│
├── apps/                          # Standalone applications
│   └── axon_recorder/             # Main recorder application
│       ├── src/                   # Application entry points
│       ├── config/                # Runtime configurations
│       ├── scripts/               # Utility scripts
│       └── test/                  # Application-level tests 
│
├── cmake/                         # Centralized CMake modules
│   └── Modules/
│
├── docker/                        # Docker configuration
│   └── scripts/
│
├── .github/                       # GitHub CI/CD
│   └── workflows/
│
├── docs/                          # Documentation
│   └── designs/                   # Design documentation
│
├── githooks/                      # Git hooks
│
├── build/                         # Build artifacts (not tracked)
│   ├── core/                      # Core libraries build output
│   ├── apps/                      # Applications build output
│   └── coverage/                  # Coverage reports
│
├── CMakeLists.txt                 # Root CMake configuration
├── Makefile                       # Primary build system
├── ARCHITECTURE.md                # System architecture
└── README.md                      # User documentation
```

## Directory Layout Overview

## Core Libraries (`core/`)

Core libraries contain **no ROS dependencies** and can be built and tested independently. They are designed as standalone C++ libraries following modern C++ best practices.

### Structure Template

Each core library follows this structure:

```
core/<library_name>/
├── src/                           # Source files (.cpp, .hpp)
│   ├── <library>.cpp              # Main implementation
│   ├── <library>.hpp              # Public header
│   └── ...                        # Additional headers/sources
├── test/                          # Unit tests
│   ├── test_<component>.cpp       # Test files (auto-discovered)
│   └── ...                        # More tests
├── CMakeLists.txt                 # Library build configuration
└── package.xml                    # (Optional) Metadata
```

### Library Descriptions

| Library | Purpose | Key Dependencies |
|---------|---------|------------------|
| **axon_logging** | Async Boost.Log with console/file/ROS sinks | Boost.Log, spdlog |
| **axon_mcap** | Thread-safe MCAP writer with compression | libmcap, Zstd, LZ4 |
| **axon_uploader** | S3 multipart upload with crash recovery | AWS SDK, SQLite |

### Build Artifacts

Core library build artifacts are placed in `build/core/` (not tracked in git):
- CMake build output
- Compiled object files and libraries
- Test executables

## Middleware (`middlewares/`)

The middleware directory contains ROS-specific code that integrates the core libraries with ROS 1/2 APIs.

### axon_recorder Package Structure

```
middlewares/src/axon_recorder/
├── src/                           # Implementation
│   ├── main.cpp                   # Entry point
│   ├── recorder_node.hpp/cpp      # Main ROS node
│   ├── state_machine.hpp/cpp      # 4-state FSM
│   ├── service_adapter.hpp/cpp    # ROS service interface
│   ├── http_callback_client.hpp/cpp  # HTTP callbacks
│   ├── config_parser.cpp          # YAML config loader
│   └── message_factory.hpp        # ROS message helpers
│
├── srv/                           # ROS service definitions
│   ├── StartRecording.srv         # Start recording service
│   ├── StopRecording.srv          # Stop recording service
│   ├── PauseRecording.srv         # Pause recording service
│   └── ...
│
├── config/                        # YAML configurations
│   └── default_config.yaml        # Default recording config
│
├── launch/                        # ROS launch files
│   └── recorder.launch.py         # Main launch file
│
├── test/                          # Comprehensive test suite
│   ├── unit/                      # Unit tests
│   ├── integration/               # Integration tests (service API)
│   ├── e2e/                       # End-to-end tests
│   ├── perf/                      # Performance benchmarks
│   ├── regression/                # Regression tests
│   └── stress/                    # Stress tests
│
├── CMakeLists.txt                 # Package build configuration
└── package.xml                    # ROS package metadata
```

### Key Components

| Component | File | Responsibility |
|-----------|------|----------------|
| **Recorder Node** | `recorder_node.hpp/cpp` | Main ROS node, coordinates all components |
| **State Machine** | `state_machine.hpp/cpp` | 4-state FSM (IDLE, READY, RECORDING, PAUSED) |
| **Service Adapter** | `service_adapter.hpp/cpp` | ROS service request/response handling |
| **HTTP Client** | `http_callback_client.hpp/cpp` | Server callbacks for lifecycle events |
| **Config Parser** | `config_parser.cpp` | YAML configuration loading |
| **Message Factory** | `message_factory.hpp` | ROS message serialization helpers |

### Build Artifacts

Middleware packages use ROS build tools (catkin for ROS1, colcon for ROS2).
Build artifacts remain within the `middlewares/` directory:
- `middlewares/ros1/build/` - ROS1 catkin build output
- `middlewares/ros1/devel/` - ROS1 development workspace
- `middlewares/ros1/install/` - ROS1 installation directory
- `middlewares/ros2/build/` - ROS2 colcon build output
- `middlewares/ros2/install/` - ROS2 installation directory
- `middlewares/ros2/log/` - ROS2 build logs

## Build System

### Root CMakeLists.txt

The root CMake file defines:
- Centralized path definitions for all modules
- Common compiler flags and warnings
- Dependency management
- Version detection (ROS 1 vs ROS 2)

### Makefile

Primary build system providing:
- `make build` - Build everything (auto-detects ROS version)
- `make test` - Run all tests
- `make debug` / `make release` - Build mode selection
- `make docker-test-*` - Docker-based testing
- `make coverage` - Code coverage reports
- `make format` / `make lint` - Code quality

### CMake Modules (`cmake/Modules/`)

| Module | Purpose |
|--------|---------|
| **AxonStdlib.cmake** | Common CMake functions (test discovery, compiler setup) |
| **AxonCoverage.cmake** | Coverage reporting utilities (lcov integration) |

## Applications (`apps/`)

The `apps/` directory contains standalone applications that combine core libraries and ROS middleware into executable programs.

### Application Structure Template

Each application follows this structure:

```
apps/<app_name>/
├── src/                           # Application entry points
│   ├── main.cpp                   # Main entry point
│   └── ...                        # Additional source files
├── config/                        # Runtime configurations
│   ├── default.yaml               # Default configuration
│   └── profiles/                  # Configuration profiles
│       ├── low_latency.yaml
│       └── high_throughput.yaml
├── scripts/                       # Utility scripts
│   ├── install.sh                 # Installation script
│   ├── setup.sh                   # Environment setup
│   └── run.sh                     # Quick start script
├── test/                          # Application-level tests
│   ├── integration/               # Integration tests
│   └── e2e/                       # End-to-end tests
├── CMakeLists.txt                 # Application build configuration
└── README.md                      # Application-specific documentation
```

### Application: axon_recorder

The main recorder application provides a complete recording solution:

```
apps/axon_recorder/
├── src/
│   ├── main.cpp                   # Entry point, argument parsing
│   ├── recorder_app.hpp/cpp       # Application orchestration
│   └── signal_handler.hpp/cpp     # Graceful shutdown handling
│
├── config/
│   ├── default.yaml               # Default recording configuration
│   └── profiles/
│       ├── minimal.yaml           # Minimal resource usage
│       ├── standard.yaml          # Standard recording profile
│       └── high_performance.yaml   # Maximum throughput
│
├── scripts/
│   ├── install.sh                 # Install application
│   ├── setup.sh                   # Configure environment
│   ├── run.sh                     # Start recorder with defaults
│   └── status.sh                  # Query recorder status
│
├── test/
│   ├── integration/               # Integration tests
│   │   ├── test_full_workflow.cpp
│   │   └── test_signal_handling.cpp
│   └── e2e/                       # End-to-end tests
│       └── test_recording_lifecycle.sh
│
├── CMakeLists.txt
└── README.md
```

### Application Responsibilities

| Responsibility | Description |
|----------------|-------------|
| **Argument Parsing** | Command-line interface for configuration |
| **Environment Setup** | ROS initialization, logging configuration |
| **Signal Handling** | Graceful shutdown on SIGINT/SIGTERM |
| **Error Recovery** | Crash recovery and state restoration |
| **User Interaction** | Status reporting, progress indicators |

### Build Artifacts

Application build artifacts are placed in `build/apps/` (not tracked in git):
- CMake build output
- Compiled executables
- Test binaries

## Build Artifacts Summary

| Component | Build Output Location | Notes |
|-----------|----------------------|-------|
| **Core Libraries** | `build/core/` | CMake build, not tracked |
| **ROS1 Middleware** | `middlewares/ros1/build/` | Catkin workspace, not tracked |
| **ROS2 Middleware** | `middlewares/ros2/build/` | Colcon workspace, not tracked |
| **Applications** | `build/apps/` | CMake build, not tracked |
| **Coverage Reports** | `build/coverage/` | LCOV reports, not tracked |

## Docker Organization

### Dockerfiles

Each ROS distribution has its own Dockerfile:
- `Dockerfile.ros1` - ROS 1 Noetic (Ubuntu 20.04)
- `Dockerfile.ros2.humble` - ROS 2 Humble (Ubuntu 22.04)
- `Dockerfile.ros2.jazzy` - ROS 2 Jazzy (Ubuntu 24.04)
- `Dockerfile.ros2.rolling` - ROS 2 Rolling (Ubuntu 24.04)

### Docker Compose Files

| File | Purpose |
|------|---------|
| `docker-compose.yml` | Base configuration |
| `docker-compose.test.yml` | Test environment setup |
| `docker-compose.perf.yml` | Performance testing environment |

### Scripts (`docker/scripts/`)

Reusable shell scripts for:
- Building Docker images
- Running tests in containers
- Setting up test environments

## CI/CD (`.github/workflows/`)

| Workflow | Purpose |
|----------|---------|
| `ci.yml` | Main CI pipeline (build + basic tests) |
| `unit-tests-cpp.yml` | C++ library unit tests |
| `integration-tests-cpp.yml` | Integration tests |
| `tests-ros.yml` | ROS-specific tests (all versions) |
| `e2e-tests.yml` | End-to-end workflow tests |
| `docker-build.yml` | Docker image build verification |

## Documentation (`docs/`)

### Design Documents (`docs/designs/`)

Design documents for individual components:

| Document | Description |
|----------|-------------|
| `edge-uploader-design.md` | S3 uploader architecture and retry logic |
| `logging-infrastructure-design.md` | Boost.Log setup and sinks |
| `metadata-injection-design.md` | MCAP metadata injection strategy |
| `middleware-plugin-architecture-design.md` | Plugin system design |
| `recording-service-api-design.md` | ROS service API specification |
| `test-design-recorder-core.md` | Testing strategy and coverage goals |
| `project-structure.md` | This file |

## Configuration (`middlewares/src/axon_recorder/config/`)

YAML configuration files define:
- Topic subscriptions and QoS settings
- Compression parameters
- Metadata injection rules
- Upload targets and retry policies
- HTTP callback endpoints

## Test Organization

### Test Types by Location

| Test Type | Location | Examples |
|-----------|----------|----------|
| **Unit** | `core/*/test/`, `middlewares/.../test/unit/` | Individual component testing |
| **Integration** | `middlewares/.../test/integration/` | Service API, state machine |
| **E2E** | `middlewares/.../test/e2e/` | Full recording workflow |
| **Performance** | `middlewares/.../test/perf/` | Benchmarking |
| **Regression** | `middlewares/.../test/regression/` | Bug reproducers |
| **Stress** | `middlewares/.../test/stress/` | Load testing |

### Test Auto-Discovery

Tests use CMake `GLOB CONFIGURE_DEPENDS` to auto-discover `test_*.cpp` files. 

## Design Principles

### 1. Separation of Concerns

- **Core libraries** have zero ROS dependencies
- **Middleware** only handles ROS integration
- Clear dependency direction: ROS → Middleware → Core

### 2. Dual ROS Support

- Single codebase supports ROS 1 and ROS 2
- Conditional compilation via `AXON_ROS1` / `AXON_ROS2` macros
- Auto-detection of ROS environment at build time

### 3. Independent Testability

- Core libraries can be tested without ROS
- Middleware tests use ROS test infrastructure
- Docker provides isolated test environments

### 4. Minimal Coupling

- Core libraries expose well-defined C++ APIs
- Communication through dependency injection
- No circular dependencies between modules

## Future Extensions

### Additional Applications

The `apps/` directory can accommodate additional standalone applications:
- Data conversion utilities (MCAP to other formats)
- Batch processing scripts (post-processing recordings)
- Analysis tools (recording inspection, statistics)

### Plugin System

See `middleware-plugin-architecture-design.md` in this directory for the planned plugin architecture.

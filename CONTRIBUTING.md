# Contributing to Axon

Thank you for your interest in contributing to Axon! This document contains essential information for all contributors.

---

## Table of Contents

- [Development Setup](#development-setup)
- [Pre-commit Hooks](#pre-commit-hooks)
- [Code Style](#code-style)
- [Development Workflow](#development-workflow)
- [Testing](#testing)
- [Building](#building)
- [Project Structure](#project-structure)
- [Reporting Issues](#reporting-issues)

---

## Development Setup

### Prerequisites

**System Dependencies:**
- CMake 3.12+
- Boost 1.71+ (`libboost-log-dev`, `libboost-filesystem-dev`, `libboost-thread-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- OpenSSL (`libssl-dev`)
- zstd (`libzstd-dev`) - optional, for MCAP compression
- lz4 (`liblz4-dev`) - optional, for MCAP compression

**ROS Environment:**
- ROS 1 Noetic OR ROS 2 (Humble/Jazzy/Rolling)

**Development Tools:**
- clang-format (required for code formatting)
- clang-format-14 or later recommended
- lcov (for coverage reports)

### Initial Setup

1. Clone the repository:
```bash
git clone git@github.com:ArcheBase/Axon.git
cd Axon
```

2. Configure git hooks:
```bash
git config core.hooksPath githooks
```

3. Source ROS environment:
```bash
# ROS 1
source /opt/ros/noetic/setup.bash

# OR ROS 2
source /opt/ros/<distro>/setup.bash
```

---

## Pre-commit Hooks

Axon uses pre-commit hooks to enforce code quality standards. These hooks run automatically before each commit.

### Setting Up Hooks

After cloning, configure git to use the shared hooks directory:

```bash
git config core.hooksPath githooks
```

### Available Hooks

| Hook | Purpose |
|------|---------|
| `pre-commit` | Checks clang-format compliance on staged C/C++ files |
| `pre-push` | Git LFS validation |

### The clang-format Hook

The pre-commit hook will:
- Only check **staged** C/C++ files (`.cpp`, `.hpp`, `.cc`, `.hh`, `.h`, `.c`)
- Exclude build directories and dependency folders
- Fail the commit if any file needs formatting

If formatting is needed, you'll see:
```
Running clang-format check...
  path/to/file.cpp needs formatting

clang-format check failed. Please run:
  git clang-format
or format files individually:
  clang-format -i <file>
```

### Formatting Your Code

**Option 1: Format staged files**
```bash
git clang-format
```

**Option 2: Format all files**
```bash
make format
```

**Option 3: Format individual files**
```bash
clang-format -i path/to/file.cpp
```

### Skipping the Hook (Not Recommended)

If you absolutely must bypass the hook:
```bash
git commit --no-verify -m "message"
```

---

## Code Style

Axon uses [clang-format](.clang-format) with a Google-based style.

### Key Style Guidelines

- **Indentation**: 2 spaces (no tabs)
- **Column limit**: 100 characters
- **Pointer alignment**: Left (`int* ptr` not `int *ptr`)
- **Namespace comments**: Enabled (`FixNamespaceComments: true`)
- **Include sorting**: Enabled with priority for axon includes

### Format Configuration

The `.clang-format` file is located at the repository root. Key settings:

```yaml
BasedOnStyle: Google
ColumnLimit: 100
PointerAlignment: Left
TabWidth: 2
UseTab: Never
```

### Include Order

Includes are sorted with the following priority:
1. Axon includes (`#include <axon/...>`)
2. Third-party includes with directory (`#include <boost/...>`)
3. Standard library includes (`#include <memory>`)
4. Relative/local includes (`#include "..."`)

---

## Development Workflow

### Branching Strategy

- `main` - Protected branch, all PRs must target this
- Feature branches - Named like `feature/your-feature` or `fix/your-bug-fix`
- Use descriptive branch names

### Commit Messages

Follow conventional commit format:

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

**Types:** `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

**Examples:**
```
feat(recorder): add pause/resume functionality
fix(uploader): resolve memory leak in retry handler
test(mcap): add thread safety tests
docs: update contributing guide
```

### Pull Request Process

1. Update your branch with the latest `main`:
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. Ensure all checks pass:
   - CI tests pass
   - Code is formatted (pre-commit hook)
   - Coverage is maintained or improved

3. Create a descriptive PR title and description

4. Request review from maintainers

5. Address review feedback

6. Once approved, squash and merge

---

## Testing

### Running Tests Locally

**Quick test (C++ libraries only):**
```bash
make test
```

**Full test suite with Docker:**
```bash
make docker-test-all
```

**Specific ROS version:**
```bash
make docker-test-ros1          # ROS 1 Noetic
make docker-test-ros2-humble   # ROS 2 Humble
make docker-test-ros2-jazzy    # ROS 2 Jazzy
make docker-test-ros2-rolling  # ROS 2 Rolling
```

### Coverage Reports

```bash
make coverage-html
open ../coverage/html/index.html
```

### Test Requirements

- All new features must have tests
- Maintain or improve code coverage
- Tests must be thread-safe where applicable
- Use mocks for external dependencies (S3, HTTP, etc.)

---

## Building

### Quick Build

```bash
make build
```

### Build Modes

```bash
make release  # Optimized build (default)
make debug    # Debug build with symbols
```

### Clean Build

```bash
make clean
make build
```

### Docker Build

```bash
make docker-build
```

---

## Project Structure

```
Axon/
├── .clang-format          # Code formatting rules
├── .github/               # GitHub Actions CI workflows
├── cmake/                 # CMake modules (ClangFormat.cmake, etc.)
├── core/                  # C++ shared libraries
│   ├── axon_logging/      # Logging infrastructure
│   ├── axon_mcap/         # MCAP writer wrapper
│   └── axon_uploader/     # S3 edge uploader
├── docs/                  # Design documents
├── docker/                # Docker build files
├── githooks/              # Shared git hooks
├── middlewares/           # ROS packages
│   └── src/axon_recorder/ # Main ROS recorder package
└── scripts/               # Utility scripts
```

### C++ Libraries

| Library | Purpose |
|---------|---------|
| `axon_logging` | Structured logging with console/file/ROS sinks |
| `axon_mcap` | MCAP writer wrapper and validation |
| `axon_uploader` | S3 upload with retry and state management |

### ROS Package

- **axon_recorder**: Main ROS node that integrates all C++ libraries
  - Services: `CachedRecordingConfig`, `RecordingControl`, `RecordingStatus`, `IsRecordingReady`
  - Topics: Subscribes to configured ROS topics for recording

---

## Coding Guidelines

### C++ Best Practices

- Use RAII for resource management
- Prefer `std::unique_ptr` and `std::shared_ptr` over raw pointers
- Use `constexpr` and `const` where possible
- Avoid exceptions in performance-critical paths
- Use lock-free data structures (SPSC queues) for producer-consumer patterns
- Document thread safety guarantees

### Thread Safety

- Document which functions are thread-safe
- Use `std::mutex` with `std::lock_guard` or `std::unique_lock`
- For SPSC queues, use lock-free implementations
- Avoid deadlock by establishing lock ordering

### Memory Management

- Prefer stack allocation over heap
- Use smart pointers for ownership
- Be aware of move semantics - use `std::move` where appropriate
- Avoid circular references with `shared_ptr`

---

## Reporting Issues

When reporting bugs or requesting features:

1. Search existing issues first
2. Use the appropriate issue template
3. Provide:
   - Steps to reproduce
   - Expected vs actual behavior
   - Environment (OS, ROS version, branch)
   - Logs if applicable

---

## Questions?

- Check [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- Check [README.md](README.md) for usage information
- Check `docs/` for detailed design documents
- Open a GitHub Discussion for questions

---

Thank you for contributing to Axon!

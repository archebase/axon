# Contributing to Axon

Thank you for your interest in contributing to Axon! This document provides essential information for all contributors.

**[中文版文档](CONTRIBUTING_ZH.md)** | English Version

## Table of Contents

- [Code of Conduct](#code-of-conduct) - See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) for details
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Contributing Guidelines](#contributing-guidelines)
- [Development Workflow](#development-workflow)
- [Code Style](#code-style)
- [Testing](#testing)
- [Building](#building)
- [Submitting Changes](#submitting-changes)
- [Reporting Issues](#reporting-issues)
- [Community Guidelines](#community-guidelines)

---

## Code of Conduct

**See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)** for the full Code of Conduct.

---

## Getting Started

### Ways to Contribute

There are many ways to contribute to Axon:

1. **Report bugs** - Submit bug reports to help us improve
2. **Suggest features** - Request features that would be useful to you
3. **Write code** - Fix bugs or implement features
4. **Improve documentation** - Help make Axon easier to understand
5. **Review pull requests** - Help review and test contributions
6. **Answer questions** - Help other users on GitHub Discussions
7. **Share your use case** - Tell us how you're using Axon

### First-Time Contributors

We welcome first-time contributors! Start with:
- Issues labeled `good first issue`
- Issues labeled `help wanted`
- Documentation improvements
- Bug fixes

---

## Development Setup

### Prerequisites

**System Dependencies:**
- CMake 3.12+
- GCC 7+ or Clang 6+ (with C++17 support)
- Boost 1.71+ (`libboost-all-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- OpenSSL (`libssl-dev`)
- zstd (`libzstd-dev`) - optional, for MCAP compression
- lz4 (`liblz4-dev`) - optional, for MCAP compression

**ROS Environment:**
- ROS 1 Noetic OR ROS 2 (Humble/Jazzy/Rolling)

**Development Tools:**
- clang-format (required for code formatting)
- clang-format-14 or later recommended
- clang-tidy (optional, for static analysis)
- lcov (for coverage reports)
- cppcheck (optional, for static analysis)

### Initial Setup

1. **Fork and clone the repository:**
```bash
# Fork the repository on GitHub first
git clone --recurse-submodules https://github.com/YOUR_USERNAME/Axon.git
cd Axon
git remote add upstream https://github.com/ArcheBase/Axon.git
```

2. **Configure git hooks:**
```bash
git config core.hooksPath githooks
```

3. **Source ROS environment:**
```bash
# ROS 1
source /opt/ros/noetic/setup.bash

# OR ROS 2
source /opt/ros/<distro>/setup.bash
```

4. **Install dependencies:**
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    libyaml-cpp-dev \
    libssl-dev \
    libzstd-dev \
    liblz4-dev \
    clang-format \
    clang-tidy \
    lcov \
    cppcheck
```

---

## Contributing Guidelines

### License

By contributing to Axon, you agree that your contributions will be licensed under the **Mulan PSL v2** License.

All source files must include the following license header:

```cpp
// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.
```

### We Welcome

- Bug fixes
- New features (please discuss in an issue first)
- Performance improvements
- Documentation improvements
- Test additions
- Code refactoring (maintaining functionality)

### We Generally Do Not Accept

- Undiscussed breaking changes
- Features that don't align with project goals
- Large refactors without prior approval
- Changes that reduce code coverage
- Dependencies on proprietary software

### Design Philosophy

Axon follows these design principles:
1. **Task-centric**: One task = one MCAP file with full lifecycle management
2. **Lock-free**: SPSC queues for zero-copy message handling
3. **Fleet-ready**: Server-controlled recording via HTTP RPC API
4. **Crash-resilient**: State persistence and recovery in S3 uploader
5. **Plugin-based**: Middleware-agnostic core with ROS1/ROS2 plugins

Please ensure your contributions align with these principles.

---

## Development Workflow

### Branching Strategy

- **`main`** - Protected branch, all PRs must target this
- **`develop`** - Feature integration branch (if applicable)
- **Feature branches** - Named like `feature/your-feature` or `fix/your-bug-fix`
- **Release branches** - Named like `release/vX.Y.Z`

### Creating a Feature Branch

```bash
git checkout main
git fetch upstream
git rebase upstream/main
git checkout -b feature/your-feature-name
```

### Commit Message Format

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, etc.)
- `refactor`: Code refactoring
- `perf`: Performance improvement
- `test`: Adding or updating tests
- `chore`: Maintenance tasks
- `ci`: CI/CD changes

**Scopes:**
- `recorder`: Main recorder application
- `mcap`: MCAP writer library
- `uploader`: S3 uploader library
- `logging`: Logging library
- `ros1`: ROS1 middleware
- `ros2`: ROS2 middleware
- `http`: HTTP RPC API
- `test`: Test code
- `docs`: Documentation
- `build`: Build system

**Examples:**
```
feat(recorder): add pause/resume functionality

Implement pause and resume operations for active recording sessions.
Maintains SPSC queue state during pause and automatically
resumes message consumption on resume.

Closes #123
```

```
fix(uploader): resolve memory leak in retry handler

The retry handler was not properly cleaning up failed upload
attempts, causing memory to accumulate over time. This fix
ensures proper resource cleanup in all error paths.

Fixes #145
```

### Making Changes

1. Write code following our [Code Style](#code-style)
2. Add tests for new features
3. Run tests locally
4. Format your code
5. Commit with conventional commit messages
6. Push to your fork
7. Create a pull request

---

## Code Style

### Pre-commit Hooks

Axon uses pre-commit hooks to enforce code quality standards:

```bash
# Configure hooks
git config core.hooksPath githooks

# Pre-commit hooks will:
# - Check clang-format compliance for staged files
# - Fail commit if formatting is needed
```

**If formatting is needed:**
```bash
# Format staged files
git clang-format

# OR format all files
make format

# OR format single file
clang-format -i path/to/file.cpp
```

**Skip hooks (not recommended):**
```bash
git commit --no-verify -m "message"
```

### Formatting Guidelines

Axon uses [clang-format](.clang-format) based on Google style:

| Setting | Value |
|---------|-------|
| BasedOnStyle | Google |
| ColumnLimit | 100 |
| PointerAlignment | Left (`int* ptr`) |
| TabWidth | 2 |
| UseTab | Never |

### Header Include Order

Order includes by priority:

1. **Axon includes** - `#include <axon/...>`
2. **Third-party includes** - `#include <boost/...>`
3. **Standard library** - `#include <memory>`
4. **Local includes** - `#include "..."`

Example:
```cpp
// Axon includes
#include <axon/axon_log_init.hpp>

// Third-party includes
#include <boost/asio.hpp>
#include <mcap/writer.hpp>

// Standard library
#include <memory>
#include <string>

// Local includes
#include "my_header.hpp"
```

### C++ Best Practices

**General:**
- Use RAII for resource management
- Prefer `std::unique_ptr` and `std::shared_ptr` over raw pointers
- Use `constexpr` and `const` where possible
- Use `override` keyword for overridden virtual functions
- Mark functions `noexcept` where appropriate
- Pass large objects by `const` reference
- Use `enum class` instead of plain `enum`

**Thread Safety:**
- Document thread safety guarantees
- Use `std::mutex` with `std::lock_guard` or `std::unique_lock`
- Use lock-free implementations for SPSC queues
- Avoid deadlocks by establishing lock ordering
- Prefer atomic operations over locks when possible

**Memory Management:**
- Prefer stack allocation over heap
- Use smart pointers for ownership
- Be aware of move semantics - use `std::move` where appropriate
- Avoid circular references with `shared_ptr`
- Use `std::make_unique` and `std::make_shared`

**Performance:**
- Avoid premature optimization
- Profile before optimizing
- Use appropriate data structures
- Minimize allocations in hot paths
- Consider cache locality

### Documentation

- All public APIs must have documentation comments
- Use Doxygen-style comments (`///` or `/** */`)
- Document thread safety guarantees
- Include usage examples for complex APIs
- Keep comments in sync with code changes

Example:
```cpp
/// @brief Thread-safe MCAP writer wrapper
///
/// This class provides a thread-safe interface for writing messages
/// to MCAP files. It uses lock-free SPSC queues for each topic to
/// achieve zero-copy message handling.
///
/// @threadsafe{All methods are thread-safe}
/// @note Writer must not be destroyed while operations are in progress
class McapWriterWrapper {
public:
    /// @brief Write a message to the MCAP file
    /// @param topic Topic name
    /// @param message Message data to write
    /// @param timestamp Message timestamp in nanoseconds
    /// @return true if write succeeded, false otherwise
    bool write(const std::string& topic,
               const std::vector<uint8_t>& message,
               uint64_t timestamp);
};
```

---

## Testing

### Testing Philosophy

- **Test-Driven Development**: Write tests before or alongside code
- **Coverage**: Maintain >80% code coverage
- **Isolation**: Tests should be independent and runnable in any order
- **Speed**: Unit tests should be fast (< 0.1s each)
- **Clarity**: Tests should serve as documentation

### Running Tests Locally

**Quick tests (C++ core libraries only):**
```bash
make test
```

**Full test suite using Docker (no local ROS needed):**
```bash
make docker-test-all
```

**Specific test categories:**
```bash
make test-core                    # C++ core library tests
make test-mcap                    # MCAP writer tests
make test-uploader                # Uploader tests
make docker-test-ros1             # ROS1 tests
make docker-test-ros2-humble      # ROS2 Humble tests
make docker-test-ros2-jazzy       # ROS2 Jazzy tests
make docker-test-ros2-rolling     # ROS2 Rolling tests
```

**Run specific tests:**
```bash
cd build
ctest -R test_mcap_writer -V
ctest -R test_state_machine --output-on-failure
```

### Test Structure

```
axon_<module>/
├── src/
│   └── ...
├── include/axon/
│   └── ...
└── test/
    ├── unit/                     # Unit tests
    │   ├── test_*.cpp
    │   └── test_*.hpp
    ├── integration/              # Integration tests
    │   └── test_*.cpp
    └── mocks/                   # Test doubles
        └── *_mock*.hpp
```

### Writing Tests

**Unit tests:**
```cpp
#include <gtest/gtest.h>
#include <axon/my_class.hpp>

namespace axon::testing {

TEST(MyClassTest, ConstructionCreatesValidObject) {
    MyClass obj(42);
    EXPECT_EQ(obj.value(), 42);
}

TEST(MyClassTest, SetValueUpdatesValue) {
    MyClass obj(0);
    obj.set_value(100);
    EXPECT_EQ(obj.value(), 100);
}

TEST(MyClassDeathTest, NullPointerThrows) {
    MyClass obj(nullptr);
    EXPECT_THROW(obj.use(), std::runtime_error);
}

}  // namespace axon::testing
```

**Test naming:**
- Use `TEST(TestSuite, TestName)` format
- TestSuite: Class/module being tested
- TestName: What is being tested (using `Should_ExpectedBehavior_When_StateUnderTest` format)

### Coverage Reports

```bash
make coverage-html
open ../coverage/html/index.html
```

**Coverage targets:**
- Core libraries: >80%
- Critical paths (state machine, recording): >90%
- New code: Tests required before merge

### Test Requirements

**For PRs:**
- All tests must pass
- New features must include tests
- Coverage should not decrease
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
make debug    # Debug build with symbols (-g -O0)
```

### Clean Build

```bash
make clean
make build
```

### Build Individual Components

```bash
make build-core                   # C++ core libraries
make build-ros1                  # ROS1 middleware
make build-ros2                  # ROS2 middleware
make app                         # Main applications
```

### Docker Build

```bash
make docker-build
```

---

## Submitting Changes

### Pull Request Process

1. **Update your branch:**
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. **Ensure all checks pass:**
   - CI tests pass
   - Code is formatted (pre-commit hooks)
   - Coverage is maintained or improved
   - Documentation is updated

3. **Create pull request:**
   - Use descriptive title (conventional commit format)
   - Fill out the PR template
   - Link related issues
   - Add `closes #123` or `fixes #123` if applicable

4. **PR description template:**
   ```markdown
   ## Summary
   Brief description of changes

   ## Motivation
   Why this change is needed

   ## Changes
   Detailed technical description with file links

   ## Testing
   - [ ] Unit tests pass
   - [ ] Integration tests pass
   - [ ] Manual testing completed

   ## Checklist
   - [ ] Code follows style guide
   - [ ] Self-review completed
   - [ ] Comments added for complex code
   - [ ] Documentation updated
   - [ ] No new warnings generated
   - [ ] Tests added/updated
   - [ ] All tests pass
   ```

5. **Request review:**
   - Tag relevant maintainers
   - Address review feedback promptly
   - Keep PRs focused (one feature per PR)

6. **After approval:**
   - Squash commits if needed
   - Ensure CI passes
   - Wait for maintainer to merge

### Review Process

- **Automated checks**: CI runs tests, linters, and coverage
- **Peer review**: At least one maintainer must approve
- **Review timeline**: Expect reviews within 3-5 business days
- **Iteration**: Address feedback and update your PR

### Merge Criteria

PRs are merged when:
- All CI checks pass
- At least one maintainer approval
- No unresolved review comments
- Coverage is maintained
- Documentation is complete

---

## Project Structure

### Directory Layout

```
Axon/
├── .clang-format              # Code formatting rules
├── .github/                   # GitHub Actions CI workflows
│   └── workflows/
├── cmake/                     # CMake modules
│   ├── ClangFormat.cmake
│   └── FindMcap.cmake
├── core/                      # C++ shared libraries
│   ├── axon_logging/          # Logging infrastructure
│   ├── axon_mcap/             # MCAP writer wrapper
│   └── axon_uploader/         # S3 edge uploader
├── docs/                      # Documentation
│   └── designs/               # Design documents
├── docker/                    # Docker files
├── githooks/                  # Shared git hooks
├── middlewares/               # ROS middleware plugins
│   ├── ros1/                  # ROS1 plugin
│   └── ros2/                  # ROS2 plugin
├── apps/                      # Main applications
│   ├── axon_recorder/         # HTTP RPC recorder
│   └── plugin_example/        # Plugin example
├── tools/                     # Tools and utilities
│   └── axon_panel/            # Web control panel
└── scripts/                   # Utility scripts
```

### Core Libraries

| Library | Purpose | Key Classes |
|---------|---------|-------------|
| `axon_logging` | Structured logging | `axon_log_init`, `axon_console_sink`, `axon_file_sink` |
| `axon_mcap` | MCAP operations | `McapWriterWrapper`, `McapValidator` |
| `axon_uploader` | S3 uploading | `EdgeUploader`, `S3Client`, `UploadQueue` |

### Middleware Plugins

| Plugin | ROS Version | Purpose |
|--------|-------------|---------|
| `ros1_plugin` | ROS 1 Noetic | ROS1 integration |
| `ros2_plugin` | ROS 2 Humble/Jazzy/Rolling | ROS2 integration |

---

## Reporting Issues

### Before Reporting

1. **Search existing issues** - Check if the problem has already been reported
2. **Check documentation** - Review [README.md](README.md) and design documents
3. **Try latest version** - Your issue may already be fixed

### Bug Reports

Use the bug report template and include:

**Required information:**
- Axon version
- ROS distribution (Noetic/Humble/Jazzy/Rolling)
- Operating system and version
- Steps to reproduce
- Expected vs actual behavior
- Relevant logs
- Minimal reproducible example

**Example:**
```markdown
## Description
Recorder crashes when finishing recording after pause/resume cycle

## Environment
- Axon version: v0.2.0
- ROS: Humble
- OS: Ubuntu 22.04

## Steps to Reproduce
1. Start recording: `POST /rpc/config` then `POST /rpc/begin`
2. Pause recording: `POST /rpc/pause`
3. Resume recording: `POST /rpc/resume`
4. Finish recording: `POST /rpc/finish`
5. Crash occurs

## Expected Behavior
Recording finishes cleanly, MCAP file is finalized

## Actual Behavior
Segmentation fault in `worker_thread_pool.cpp:142`

## Logs
[Include relevant log output]

## Additional Context
Works fine without pause/resume cycle
```

### Feature Requests

Use the feature request template and include:
- **Problem statement**: What problem does this solve?
- **Proposed solution**: How should it work?
- **Alternatives considered**: What other approaches did you consider?
- **Additional context**: Use cases, examples, mockups

### Security Issues

**Please do NOT report security issues publicly.**

Report security issues via email to: security@archebase.com

---

## Community Guidelines

### Communication Channels

- **GitHub Issues**: Bug reports and feature requests
- **GitHub Discussions**: Questions and general discussions
- **Pull Requests**: Code contributions
- **Email**: contact@archebase.com for administrative matters

### Getting Help

1. **Search first**: Check existing issues and discussions
2. **Be specific**: Include code, error messages, and detailed environment info
3. **Be patient**: Maintainers contribute their time voluntarily
4. **Be respectful**: Treat everyone with respect

### Recognition

Contributors will:
- Be listed in [CONTRIBUTORS.md](CONTRIBUTORS.md)
- Mentioned in release notes for significant contributions
- Eligible for maintainer status after consistent contributions

---

## Design Philosophy

### Core Principles

1. **Task-centric**: One task = one MCAP file with full lifecycle management
2. **Lock-free**: SPSC queues for zero-copy message handling
3. **Fleet-ready**: Server-controlled recording via HTTP RPC API
4. **Crash-resilient**: State persistence and recovery in S3 uploader
5. **Plugin-based**: Middleware-agnostic core with ROS1/ROS2 plugins

### Architectural Decisions

- **HTTP over ROS services**: Reduced coupling, enables web interface
- **SPSC queues**: Lock-free, cache-friendly message passing
- **State machine**: Explicit state transitions prevent invalid operations
- **MCAP format**: Modern flexible container for robotics data

---

## Additional Resources

### Documentation

- [README.md](README.md) - Project overview and usage
- [ARCHITECTURE.md](ARCHITECTURE.md) - System architecture
- [docs/designs/rpc-api-design.md](docs/designs/rpc-api-design.md) - HTTP RPC API specification
- [docs/designs/frontend-design.md](docs/designs/frontend-design.md) - Web panel architecture

### External References

- [Mulan PSL v2 License](LICENSE)
- [ROS 1 Documentation](http://wiki.ros.org/noetic)
- [ROS 2 Documentation](https://docs.ros.org/)
- [MCAP Format](https://mcap.dev/)
- [Conventional Commits](https://www.conventionalcommits.org/)

---

## License

By contributing to Axon, you agree that your contributions will be licensed under the **Mulan PSL v2** License. See [LICENSE](LICENSE) for details.

---

## Questions?

- Check existing [GitHub Issues](https://github.com/ArcheBase/Axon/issues)
- Start a [GitHub Discussion](https://github.com/ArcheBase/Axon/discussions)
- See [ARCHITECTURE.md](ARCHITECTURE.md) for system design
- See [README.md](README.md) for usage information

---

Thank you for contributing to Axon! Your contributions make Axon better for everyone. 🚀

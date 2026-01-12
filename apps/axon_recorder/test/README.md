# Axon Recorder Tests

This directory contains unit and integration tests for the Axon recorder application.

## Test Structure

```
test/
├── CMakeLists.txt           # Test build configuration
├── unit/                    # Unit tests (no runtime ROS required)
│   ├── test_helpers.hpp    # Test helper utilities
│   ├── test_state_machine.cpp
│   ├── test_state_transaction_guard.cpp
│   ├── test_task_config.cpp
│   ├── test_config_parser.cpp
│   ├── test_http_callback_client.cpp
│   ├── test_spsc_queue.cpp
│   ├── test_worker_thread_pool.cpp
│   └── test_message_factory_standalone.cpp
└── integration/             # Integration tests (require core libraries)
    └── test_recorder_integration.cpp
```

## Building Tests

### From Project Root

```bash
# Build all (including tests)
mkdir -p build && cd build
cmake .. -DAXON_BUILD_TESTS=ON
make -j$(nproc)

# Build only tests
make -j$(nproc) test_state_machine test_config_parser test_http_callback_client
```

### From Test Directory

```bash
# From apps/axon_recorder/test
mkdir -p build && cd build
cmake .. -DAXON_BUILD_TESTS=ON -DAXON_BUILD_WITH_CORE=ON
make -j$(nproc)
```

## Running Tests

### Run All Tests

```bash
# Using CTest
cd build
ctest --output-on-failure

# Run specific test
./test_state_machine
```

### Run Tests by Label

```bash
# Run only unit tests
ctest -L unit --output-on-failure

# Run only state machine tests
ctest -L state --output-on-failure

# Run only config tests
ctest -L config --output-on-failure

# Run only integration tests
ctest -L integration --output-on-failure
```

### Run Specific Test

```bash
# Run a specific test executable
./test_state_machine

# Run with verbose output
./test_state_machine --gtest_verbose

# Run specific test case
./test_state_machine --gtest_filter=StateMachineTest.ValidTransitionIdleToReady
```

## Coverage Report

To generate a code coverage report:

```bash
# Build with coverage enabled
mkdir -p build && cd build
cmake .. -DAXON_BUILD_TESTS=ON -DAXON_ENABLE_COVERAGE=ON
make -j$(nproc)

# Run tests to generate coverage data
ctest

# Generate coverage report
make coverage

# View the report
open coverage_html/index.html  # macOS
xdg-open coverage_html/index.html  # Linux
```

### Coverage Clean

```bash
make coverage-clean
```

## Test Descriptions

### Unit Tests

| Test File | Description | Tests Count |
|-----------|-------------|-------------|
| `test_state_machine.cpp` | StateManager state transitions | ~40 |
| `test_state_transaction_guard.cpp` | StateTransactionGuard RAII wrapper | ~7 |
| `test_task_config.cpp` | TaskConfig validation and serialization | ~15 |
| `test_config_parser.cpp` | YAML configuration parsing | ~40 |
| `test_http_callback_client.cpp` | HTTP callback notifications | ~30 |
| `test_spsc_queue.cpp` | Lock-free SPSC/MPSC queues | ~25 |
| `test_worker_thread_pool.cpp` | Per-topic worker threads | ~20 |
| `test_message_factory_standalone.cpp` | Message factory registry pattern | ~10 |

### Integration Tests

| Test File | Description | Tests Count |
|-----------|-------------|-------------|
| `test_recorder_integration.cpp` | Full AxonRecorder workflow | ~25 |

## CI/CD Integration

### GitHub Actions Example

```yaml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Build
        run: |
          mkdir -p build && cd build
          cmake .. -DAXON_BUILD_TESTS=ON
          make -j$(nproc)
      - name: Run Tests
        run: |
          cd build
          ctest --output-on-failure
      - name: Coverage
        run: |
          cd build
          cmake .. -DAXON_ENABLE_COVERAGE=ON
          make coverage
```

## Troubleshooting

### Tests Not Found

If tests are not being built, check:

```bash
# Ensure AXON_BUILD_TESTS is ON
cmake .. -DAXON_BUILD_TESTS=ON

# Check if core libraries are found
cmake .. -DAXON_BUILD_WITH_CORE=ON
```

### Missing Dependencies

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get install libgtest-dev libyaml-cpp-dev libboost-all-dev libssl-dev

# Install dependencies (macOS)
brew install gtest yaml-cpp boost openssl
```

### Linker Errors

If you encounter linker errors related to `axon_mcap`, `axon_logging`, or `axon_utils`:

```bash
# Ensure AXON_BUILD_WITH_CORE is enabled
cmake .. -DAXON_BUILD_TESTS=ON -DAXON_BUILD_WITH_CORE=ON

# Or manually specify paths
cmake .. -DAXON_MCAP_DIR=/path/to/axon_mcap
```

## Adding New Tests

1. Create test file in `unit/` or `integration/`
2. Add to `test/CMakeLists.txt`:

```cmake
# For unit tests
add_axon_test(test_my_feature
    SOURCES
        ${TEST_SOURCE_DIR}/test_my_feature.cpp
)

# For integration tests (requires linking additional source files)
add_executable(test_my_integration
    ${INTEGRATION_TEST_SOURCE_DIR}/test_my_integration.cpp
    ../src/required_source.cpp
)
# ... target_link_libraries etc
```

3. Rebuild:

```bash
cd build
cmake ..
make -j$(nproc)
```

## Disabled Tests

The following test file has been disabled due to architectural changes:

- `test_topic_manager.cpp` - Depends on removed `TopicManager` and `RosInterface` classes. The new plugin architecture replaces this functionality. To enable, rewrite using the plugin interface.

See [test-design-recorder-core.md](../../../docs/designs/test-design-recorder-core.md) for the full test plan.

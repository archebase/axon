# Test Suite

This directory contains the test suite for the Axon project by ArcheBase.

## Test Structure

- **Rust Tests**: Unit tests for the Rust bridge library (`src/bridge/src/lib.rs`)
- **C++ Unit Tests**: Tests for core C++ components
  - `test_config_parser.cpp`: Configuration parser tests
  - `test_batch_manager.cpp`: Batch manager tests
  - `test_arrow_builder.cpp`: Arrow builder tests
- **Integration Tests**: End-to-end tests (`integration/`)

## Running Tests

### Rust Tests

```bash
cd src/bridge
cargo test
```

### C++ Tests

```bash
mkdir -p build
cd build
cmake ../test
make
ctest --output-on-failure
```

### All Tests

```bash
# Run Rust tests
cd src/bridge && cargo test && cd ../..

# Build and run C++ tests
mkdir -p build && cd build
cmake ../test && make && ctest --output-on-failure
```

## Test Coverage

### Current Coverage

- ✅ Rust bridge library (FFI functions, error handling)
- ✅ Configuration parser (YAML loading, validation, saving)
- ✅ Batch manager (batch collection, flushing, async writes)
- ✅ Arrow builder (type-specific builders, reset functionality)

### Missing Coverage

- ⚠️ Message converter (needs ROS message type mocks)
- ⚠️ ROS interface abstraction (needs ROS 1/2 test environments)
- ⚠️ End-to-end integration tests (needs ROS environment)
- ⚠️ Performance benchmarks

## Adding New Tests

1. **Rust Tests**: Add to `#[cfg(test)]` module in the relevant `.rs` file
2. **C++ Tests**: Create new test file in `test/cpp/` and add to `test/CMakeLists.txt`
3. **Integration Tests**: Add scripts to `test/integration/`

## CI/CD

Tests are automatically run on push/PR via GitHub Actions (`.github/workflows/tests.yml`).


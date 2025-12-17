# Test Suite

This directory contains the test suite for the Axon project by ArcheBase.

## Test Structure

- **C FFI Tests (Rust)**: Unit tests for the C FFI library (`c/src/lib.rs`)
- **C++ Unit Tests**: Tests for core C++ components
  - `test_config_parser.cpp`: Configuration parser tests
  - `test_batch_manager.cpp`: Batch manager tests
  - `test_arrow_builder.cpp`: Arrow builder tests
- **Integration Tests**: End-to-end tests (`integration/`)

## Running Tests

### C FFI Tests (Rust)

```bash
cd c
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
# Run C FFI tests
cd c && cargo test && cd ..

# Build and run C++ tests
mkdir -p build && cd build
cmake ../test && make && ctest --output-on-failure
```

### Using Makefile

```bash
# Run all tests
make test

# Run only Rust/C FFI tests
make rust-test

# Run only C++ tests
make cpp-test
```

## Test Coverage

### Current Coverage

- ✅ C FFI library (FFI functions, error handling, Lance operations)
- ✅ Configuration parser (YAML loading, validation, saving)
- ✅ Batch manager (batch collection, flushing, async writes)
- ✅ Arrow builder (type-specific builders, reset functionality)

### Missing Coverage

- ⚠️ Message converter (needs ROS message type mocks)
- ⚠️ ROS interface abstraction (needs ROS 1/2 test environments)
- ⚠️ End-to-end integration tests (needs ROS environment)
- ⚠️ Performance benchmarks

## Adding New Tests

1. **Rust/C FFI Tests**: Add to `#[cfg(test)]` module in `c/src/*.rs` files
2. **C++ Tests**: Create new test file in `test/cpp/` and add to `test/CMakeLists.txt`
3. **Integration Tests**: Add scripts to `test/integration/`

## CI/CD

Tests are automatically run on push/PR via GitHub Actions (`.github/workflows/tests.yml`).

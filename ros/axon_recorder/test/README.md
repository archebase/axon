# Axon Recorder Tests

## Test Organization

Tests for the Axon project are organized as follows:

### Core C++ Library Tests (cpp/axon_arrow/test/)

Unit tests for the core C++ library (Arrow builders, batch manager, config parser) are located in:
```
cpp/axon_arrow/test/
├── test_arrow_builder.cpp
├── test_batch_manager.cpp
└── test_config_parser.cpp
```

Run these tests with:
```bash
cd cpp
make test
```

### ROS Integration Tests (ros/axon_recorder/test/)

ROS-specific integration tests are located here:
```
ros/axon_recorder/test/
├── integration/
│   └── test_rust_bridge.sh    # Rust FFI bridge integration test
└── README.md                   # This file
```

Run integration tests with:
```bash
cd ros
make test
```

## Running All Tests

To run all tests (both C++ unit tests and integration tests):

```bash
# From project root
cd cpp && make test
cd ../ros && make test
```

Or using Docker:
```bash
cd ros
make docker-test-all
```

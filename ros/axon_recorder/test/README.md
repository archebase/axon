# Axon Recorder Tests

## Test Organization

Tests for the Axon project are organized as follows:

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

To run all tests:

```bash
# From ros directory
cd ros
make test
```

Or using Docker:
```bash
cd ros
make docker-test-all
```

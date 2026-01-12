# E2E Tests for Axon Recorder

This directory contains end-to-end tests for the axon_recorder application.

## Overview

The E2E tests verify the complete recording workflow from start to finish:

1. **Process Management**: Starting and stopping the recorder
2. **HTTP API**: Testing all REST endpoints
3. **Recording Workflow**: Cache config → Start → Pause → Resume → Stop
4. **File Generation**: MCAP file and sidecar JSON verification
5. **Metadata Validation**: Checking task metadata in output files

## Test Structure

```
e2e/
├── run_e2e_tests.sh      # Main E2E test script
├── run_docker_e2e.sh     # Docker-based E2E test runner
├── README.md             # This file
└── test_data/            # Generated during test execution
    ├── recordings/       # MCAP files
    ├── recorder.log      # Recorder output
    └── stats.json        # Statistics
```

## Prerequisites

### Local Testing

```bash
# Build the project
cd /path/to/Axon
make build

# Install dependencies (if not already installed)
sudo apt-get install curl python3
```

### Docker Testing

```bash
# Install Docker
sudo apt-get install docker.io
```

## Running Tests

### Local Environment

```bash
# Run all E2E tests
cd apps/axon_recorder/test/e2e
./run_e2e_tests.sh
```

### Docker Environment

```bash
# Run in ROS2 Humble environment
cd apps/axon_recorder/test/e2e
./run_docker_e2e.sh

# Specify ROS version
ROS_VERSION=rolling ./run_docker_e2e.sh
```

## Test Coverage

### HTTP API Tests

| Test | Endpoint | Description |
|------|----------|-------------|
| `test_health_check` | `GET /health` | Verify recorder is running |
| `test_cache_config` | `POST /api/v1/config/cache` | Cache task configuration |
| `test_start_recording` | `POST /api/v1/recording/start` | Start recording |
| `test_recording_status` | `GET /api/v1/recording/status` | Check recording state |
| `test_pause_recording` | `POST /api/v1/recording/pause` | Pause recording |
| `test_resume_recording` | `POST /api/v1/recording/resume` | Resume recording |
| `test_stop_recording` | `POST /api/v1/recording/stop` | Stop recording |

### File Verification Tests

| Test | Description |
|------|-------------|
| `verify_mcap_file` | Check MCAP file exists and is not empty |
| `verify_sidecar_file` | Validate sidecar JSON and required fields |

### Sidecar Validation

The sidecar JSON file is validated for:

- **Valid JSON**: Parses correctly
- **Required fields**: `version`, `task_id`, `device_id`, `scene`, `recording_start_time`, `recording_end_time`, `checksum`
- **Task metadata**: Matches the cached configuration

## Test Flow

```
1. Setup
   ├── Create test data directory
   ├── Check recorder binary exists
   └── Generate test configuration

2. Start Recorder
   ├── Launch axon_recorder process
   ├── Wait for startup
   └── Verify process is running

3. HTTP API Tests
   ├── Health check
   ├── Cache configuration
   ├── Start recording
   ├── Check status (RECORDING)
   ├── Pause recording
   ├── Check status (PAUSED)
   ├── Resume recording
   └── Stop recording

4. File Verification
   ├── Find MCAP file
   ├── Check file size > 0
   ├── Find sidecar JSON
   ├── Validate JSON syntax
   └── Check required fields

5. Cleanup
   ├── Stop recorder process
   └── Remove test data
```

## Configuration

### Test Configuration

Tests use the following defaults:

| Variable | Default | Description |
|----------|---------|-------------|
| `HTTP_PORT` | 8080 | HTTP server port |
| `TEST_TASK_ID` | `e2e_test_<timestamp>` | Unique task ID |
| `TEST_DEVICE_ID` | `test_robot_01` | Device identifier |
| `TEST_SCENE` | `e2e_test_scene` | Scene name |

### Custom Configuration

You can modify test parameters by editing the configuration section in `run_e2e_tests.sh`:

```bash
# Test configuration
HTTP_PORT=8080
TEST_TASK_ID="e2e_test_$(date +%s)"
TEST_DEVICE_ID="test_robot_01"
TEST_SCENE="e2e_test_scene"
```

## Troubleshooting

### Recorder Fails to Start

Check the recorder log:
```bash
cat test_data/recorder.log
```

### HTTP Requests Fail

Verify the recorder is running:
```bash
curl http://localhost:8080/health
```

### MCAP File Not Created

1. Check the dataset path in test config
2. Verify disk space is available
3. Review recorder logs for errors

### Sidecar File Missing

1. Verify task config was cached
2. Check that recording was stopped properly
3. Look for errors in metadata injection

## Adding New Tests

To add a new E2E test:

1. Create a test function in `run_e2e_tests.sh`:
   ```bash
   test_my_new_feature() {
       log_info "Testing my new feature..."
       # Your test code here
       if [[ condition ]]; then
           log_info "Test passed"
           return 0
       else
           log_error "Test failed"
           return 1
       fi
   }
   ```

2. Add the test to the `tests` array:
   ```bash
   local tests=(
       "test_health_check"
       "test_my_new_feature"  # Add here
       # ... other tests
   )
   ```

3. Run the tests and verify:
   ```bash
   ./run_e2e_tests.sh
   ```

## CI/CD Integration

### GitHub Actions Example

```yaml
name: E2E Tests

on: [push, pull_request]

jobs:
  e2e:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        ros_version: [humble, jazzy, rolling]
    steps:
      - uses: actions/checkout@v3
      - name: Run E2E tests
        run: |
          cd apps/axon_recorder/test/e2e
          ROS_VERSION=${{ matrix.ros_version }} ./run_docker_e2e.sh
```

## Cleanup

To manually clean up test artifacts:

```bash
# Stop any running recorder
pkill -f axon_recorder

# Remove test data
rm -rf apps/axon_recorder/test/e2e/test_data

# Remove Docker artifacts
docker system prune -f
```

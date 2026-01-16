# Axon C++ SDK

C++ core libraries for Axon - provides MCAP recording, S3 upload, logging, and configuration parsing.

## Overview

The C++ SDK provides core functionality that is **not** ROS-specific:

- **MCAP Writer** (`axon_mcap/`) - High-performance MCAP file writer with compression support
- **S3 Uploader** (`axon_uploader/`) - Upload files to S3-compatible storage (AWS S3, MinIO)
- **Logging** (`axon_logging/`) - Structured logging infrastructure
- **Config Parser** (`axon_mcap/config_parser`) - Parse YAML configuration files

## MCAP Format

The recorder uses MCAP (append-only container) format for efficient storage and playback.

**Key features:**
- No Arrow/Lance dependencies
- Simplified data path (no schema conversion)
- Lower CPU overhead
- Compatible with Foxglove Studio and ROS bag tools

## Dependencies

### Required Dependencies

**All libraries:**
- **yaml-cpp** - YAML configuration parsing
- **GoogleTest** - Unit testing framework (for tests only)

**axon_mcap:**
- **zstd** (optional) - Zstd compression for MCAP
- **lz4** (optional) - LZ4 compression for MCAP

**axon_uploader:**
- **AWS SDK for C++** (S3, Transfer) - S3-compatible storage operations
- **OpenSSL** - TLS/SSL support
- **SQLite3** - Persistent upload state

**axon_logging:**
- **Boost.Log** - Logging backend

### Installing Dependencies

**macOS (Homebrew):**
```bash
# Core dependencies
brew install yaml-cpp zstd lz4 googletest

# For axon_uploader
brew install aws-sdk-cpp openssl sqlite3

# For axon_logging
brew install boost
```

**Ubuntu/Debian:**
```bash
sudo apt-get update

# Core dependencies
sudo apt-get install -y \
    libyaml-cpp-dev \
    libzstd-dev \
    liblz4-dev \
    libgtest-dev

# For axon_uploader (AWS SDK must be built from source)
sudo apt-get install -y \
    libssl-dev \
    libsqlite3-dev \
    libcurl4-openssl-dev

# Build AWS SDK from source (no apt packages available)
git clone --recurse-submodules --depth 1 https://github.com/aws/aws-sdk-cpp
cd aws-sdk-cpp
cmake -B build -DBUILD_ONLY="s3;transfer" -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo cmake --install build

# For axon_logging
sudo apt-get install -y \
    libboost-log-dev \
    libboost-thread-dev \
    libboost-filesystem-dev
```

## Building

### Using Make (Recommended)

```bash
cd core

# Show available targets
make help

# Build the library
make build

# Build and run tests
make test

# Clean build artifacts
make clean
```

### Using CMake Directly

```bash
# Build MCAP library
cd core/axon_mcap
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DAXON_MCAP_BUILD_TESTS=ON
cmake --build . -j$(nproc)

# Run tests
./test_mcap_writer
```

### CMake Options

#### MCAP Library (axon_mcap)

| Option | Default | Description |
|--------|---------|-------------|
| `AXON_MCAP_BUILD_TESTS` | `OFF` | Build MCAP unit tests |
| `AXON_MCAP_ENABLE_COVERAGE` | `OFF` | Enable code coverage |

#### Uploader Library (axon_uploader)

| Option | Default | Description |
|--------|---------|-------------|
| `AXON_UPLOADER_BUILD_TESTS` | `OFF` | Build uploader unit tests |
| `AXON_UPLOADER_ENABLE_COVERAGE` | `OFF` | Enable code coverage |

#### Logging Library (axon_logging)

| Option | Default | Description |
|--------|---------|-------------|
| `AXON_LOGGING_BUILD_TESTS` | `OFF` | Build logging unit tests |
| `AXON_LOGGING_ENABLE_COVERAGE` | `OFF` | Enable code coverage |

## API Documentation

### McapWriterWrapper

Thread-safe wrapper for writing MCAP files:

```cpp
#include "mcap_writer_wrapper.hpp"

using namespace axon::mcap_wrapper;

// Create writer with options
McapWriterWrapper writer;
McapWriterOptions options;
options.profile = "ros2";
options.compression = Compression::Zstd;
options.chunk_size = 4 * 1024 * 1024;  // 4MB chunks

// Open file
writer.open("/path/to/output.mcap", options);

// Register schema
std::string schema_def = "uint32 height\nuint32 width\nstring encoding\nuint8[] data";
uint16_t schema_id = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", schema_def);

// Register channel
uint16_t channel_id = writer.register_channel("/camera/image", "cdr", schema_id);

// Write messages (thread-safe)
writer.write(channel_id, timestamp_ns, timestamp_ns, data, size);

// Close file (writes footer)
writer.close();
```

### ConfigParser

Parses YAML configuration files:

```cpp
#include "config_parser.hpp"

axon::core::RecorderConfig config = axon::core::RecorderConfig::from_yaml("/path/to/config.yaml");

if (config.validate()) {
    // Use config
    for (const auto& topic : config.topics) {
        std::cout << "Topic: " << topic.name << std::endl;
    }
}
```

### S3Client

Upload files to S3-compatible storage (AWS S3, MinIO):

```cpp
#include "s3_client.hpp"

using namespace axon::uploader;

// Configure S3 client
S3Config config;
config.endpoint_url = "http://localhost:9000";  // MinIO endpoint
config.bucket = "my-bucket";
config.region = "us-east-1";
// Credentials loaded from AWS_ACCESS_KEY_ID and AWS_SECRET_ACCESS_KEY env vars

// Create client
S3Client client(config);

// Upload file with metadata
std::map<std::string, std::string> metadata;
metadata["checksum-sha256"] = "abc123...";

auto result = client.uploadFile(
    "/path/to/local/file.mcap",
    "recordings/file.mcap",
    metadata,
    [](uint64_t transferred, uint64_t total) {
        std::cout << "Progress: " << transferred << "/" << total << std::endl;
    }
);

if (result.success) {
    std::cout << "Upload succeeded, ETag: " << result.etag << std::endl;
} else {
    std::cerr << "Upload failed: " << result.error_message << std::endl;
}
```

## Testing

Run MCAP writer tests:
```bash
cd core/axon_mcap/build
./test_mcap_writer
```

## Usage

The C++ SDK is used by:
- **ROS packages** (`ros/src/axon_recorder/`) for ROS 1 and ROS 2 integration
- Can be used **standalone** for non-ROS applications

### Using in ROS Packages

The recorder CMakeLists.txt includes axon_mcap as a subdirectory:

```cmake
add_subdirectory(${PROJECT_ROOT}/core/axon_mcap ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
target_link_libraries(your_target axon_mcap)
```

## See Also

- [ROS Recorder Package](../ros/src/axon_recorder/README.md)
- [Main README](../README.md)
- [MCAP Specification](https://mcap.dev/)
- [Foxglove Studio](https://foxglove.dev/)

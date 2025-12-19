# Axon C++ SDK

C++ core libraries for Axon - provides MCAP recording, configuration parsing, and message utilities.

## Overview

The C++ SDK provides core functionality that is **not** ROS-specific:

- **MCAP Writer** (`axon_mcap/`) - High-performance MCAP file writer with compression support
- **Config Parser** (`axon_arrow/config_parser`) - Parse YAML configuration files
- **Message Utilities** (`axon_arrow/`) - Message introspection and conversion utilities

## Directory Structure

```
cpp/
├── Makefile                              # Build system
├── README.md                             # This file
├── axon_mcap/                            # MCAP recording library (NEW)
│   ├── CMakeLists.txt                    # CMake build configuration
│   ├── mcap_writer_wrapper.hpp           # Thread-safe MCAP writer API
│   ├── mcap_writer_wrapper.cpp           # Implementation
│   ├── include/mcap/                     # Foxglove MCAP header-only library
│   └── test/
│       └── test_mcap_writer.cpp          # MCAP writer tests
└── axon_arrow/                           # Utility libraries (legacy Arrow code)
    ├── CMakeLists.txt                    # CMake build configuration
    ├── config_parser.cpp/.hpp            # YAML config parser (ACTIVE)
    ├── message_introspection.cpp/.hpp    # Message introspection (ACTIVE)
    ├── message_converter.cpp/.hpp        # Message conversion utilities
    ├── batch_manager.cpp/.hpp            # Arrow batch management (DEPRECATED)
    ├── schema_merger.cpp/.hpp            # Schema merging (DEPRECATED)
    ├── arrow_builder.cpp/.hpp            # Arrow array builders (DEPRECATED)
    └── test/                             # Tests
```

## Migration from Lance to MCAP

The recorder has been migrated from Lance (columnar storage) to MCAP (append-only container).
See [MIGRATION_LANCE_TO_MCAP.md](../MIGRATION_LANCE_TO_MCAP.md) for details.

**Key changes:**
- No more Arrow/Lance dependencies for recording
- Simplified data path (no schema conversion)
- Lower CPU overhead
- Compatible with Foxglove Studio and ROS bag tools

## Dependencies

### Required Dependencies

- **yaml-cpp** - YAML configuration parsing
- **zstd** (optional) - Zstd compression for MCAP
- **lz4** (optional) - LZ4 compression for MCAP
- **GoogleTest** - Unit testing framework (for tests only)

### Installing Dependencies

**macOS (Homebrew):**
```bash
brew install yaml-cpp zstd lz4 googletest
```

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libzstd-dev liblz4-dev libgtest-dev
```

## Building

### Using Make (Recommended)

```bash
cd cpp

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
cd cpp/axon_mcap
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

## Testing

Run MCAP writer tests:
```bash
cd cpp/axon_mcap/build
./test_mcap_writer
```

## Usage

The C++ SDK is used by:
- **ROS packages** (`ros/axon_recorder/`) for ROS 1 and ROS 2 integration
- Can be used **standalone** for non-ROS applications

### Using in ROS Packages

The recorder CMakeLists.txt includes axon_mcap as a subdirectory:

```cmake
add_subdirectory(${PROJECT_ROOT}/cpp/axon_mcap ${CMAKE_CURRENT_BINARY_DIR}/axon_mcap)
target_link_libraries(your_target axon_mcap)
```

## See Also

- [Migration Guide](../MIGRATION_LANCE_TO_MCAP.md)
- [ROS Recorder Package](../ros/axon_recorder/README.md)
- [Main README](../README.md)
- [MCAP Specification](https://mcap.dev/)
- [Foxglove Studio](https://foxglove.dev/)

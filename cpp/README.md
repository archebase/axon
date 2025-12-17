# Axon C++ SDK

C++ core library for Axon - provides Arrow builders, batch management, and message conversion.

## Overview

The C++ SDK provides core functionality that is **not** ROS-specific:

- **Arrow Builders** - Build Apache Arrow arrays from various data types
- **Batch Manager** - Manage batches of data for efficient writing
- **Config Parser** - Parse YAML configuration files
- **Message Converter** - Convert message data to Arrow format
- **Schema Merger** - Merge schemas from multiple topics

## Directory Structure

```
cpp/
├── Makefile                              # Build system
├── README.md                             # This file
└── axon_arrow/
    ├── CMakeLists.txt                    # CMake build configuration
    ├── cmake/
    │   └── axon_arrow-config.cmake.in    # CMake package config
    ├── arrow_builder.cpp/.hpp            # Arrow array builders
    ├── batch_manager.cpp/.hpp            # Batch management
    ├── config_parser.cpp/.hpp            # YAML config parser
    ├── message_converter.cpp/.hpp        # Message conversion
    ├── message_introspection.cpp/.hpp    # Message introspection
    ├── schema_merger.cpp/.hpp            # Schema merging
    └── test/
        ├── CMakeLists.txt                # Test build configuration
        ├── test_arrow_builder.cpp        # Arrow builder tests
        ├── test_batch_manager.cpp        # Batch manager tests
        └── test_config_parser.cpp        # Config parser tests
```

## Dependencies

All dependencies must be installed before building:

- **Apache Arrow C++** - Core dependency for Arrow format support
- **yaml-cpp** - YAML configuration parsing
- **GoogleTest** - Unit testing framework

### Installing Dependencies

**macOS (Homebrew):**
```bash
brew install apache-arrow yaml-cpp googletest
```

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install -y libarrow-dev libyaml-cpp-dev libgtest-dev
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

# Build in debug mode
make debug

# Clean build artifacts
make clean
```

### Using CMake Directly

```bash
cd cpp
mkdir build && cd build
cmake ../axon_arrow -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
ctest --output-on-failure
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `BUILD_SHARED_LIBS` | `OFF` | Build shared libraries instead of static |
| `AXON_ARROW_BUILD_TESTS` | `ON` | Build unit tests |

## API Documentation

### ArrowBuilder

Builds Apache Arrow arrays from primitive types:

```cpp
#include "arrow_builder.hpp"

axon::core::ArrowBuilderFactory factory;
auto builder = factory.create_int64_builder();
auto& typed = static_cast<TypedArrowBuilder<arrow::Int64Builder>&>(*builder);
typed.get().Append(42);
typed.get().Append(100);

std::shared_ptr<arrow::Array> array;
typed.Finish(&array);
```

### BatchManager

Manages batches of data for efficient writing:

```cpp
#include "batch_manager.hpp"

axon::core::BatchManager manager(batch_size, flush_interval_ms, write_callback);
manager.set_dataset("/path/to/dataset.lance", handle);
manager.initialize_schema(schema);
manager.start();

// Add rows
manager.add_row(arrays);

// Manual flush if needed
manager.flush();

manager.stop();
```

### ConfigParser

Parses YAML configuration files:

```cpp
#include "config_parser.hpp"

axon::core::ConfigParser parser;
axon::core::RecorderConfig config;

parser.load_from_file("/path/to/config.yaml", config);

// Validate configuration
if (config.validate()) {
    // Use config
}
```

## Testing

Run all tests:
```bash
make test
```

Run specific tests:
```bash
make test-arrow   # Arrow builder tests
make test-batch   # Batch manager tests
make test-config  # Config parser tests
```

## Usage

The C++ SDK is used by:
- **ROS packages** (`ros/axon_recorder/`) for ROS 1 and ROS 2 integration
- Can be used **standalone** for non-ROS applications

### Using as a CMake Package

After installing (`make install`), you can use the library in other CMake projects:

```cmake
find_package(axon_arrow REQUIRED)
target_link_libraries(your_target axon::axon_arrow)
```

## See Also

- [C SDK (FFI Bridge)](../c/README.md)
- [ROS Recorder Package](../ros/axon_recorder/README.md)
- [Main README](../README.md)

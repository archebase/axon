# Axon Lance FFI - C Interface

C FFI layer for writing Arrow data to Lance format from C/C++ applications.

## Overview

This crate provides a C-compatible interface to the `axon-lance` core library:

- Thin wrapper exposing C functions
- Uses Apache Arrow's C Data Interface for zero-copy data transfer
- Generated C header via cbindgen

## Structure

```
c/
├── Cargo.toml          # Rust package (depends on axon-lance)
├── cbindgen.toml       # Header generation configuration
├── build.rs            # Generates lance_bridge.h
├── README.md           # This file
├── include/
│   └── axon/
│       └── lance_bridge.h  # Generated C header
└── src/
    └── lib.rs          # FFI exports only
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│                  C++ Code                        │
│              (cpp/, ros/)                        │
└─────────────────────────────────────────────────┘
                     │
                     │ #include <axon/lance_bridge.h>
                     ▼
┌─────────────────────────────────────────────────┐
│          Generated C Header                      │
│         c/include/axon/lance_bridge.h           │
└─────────────────────────────────────────────────┘
                     │
                     │ links to
                     ▼
┌─────────────────────────────────────────────────┐
│       axon-lance-ffi (this crate)               │
│           liblance_writer_bridge.so             │
└─────────────────────────────────────────────────┘
                     │
                     │ depends on
                     ▼
┌─────────────────────────────────────────────────┐
│           axon-lance (rust/)                    │
│        Core Rust library                        │
└─────────────────────────────────────────────────┘
                     │
                     │ uses
                     ▼
┌─────────────────────────────────────────────────┐
│              Lance Library (v1.0)               │
└─────────────────────────────────────────────────┘
```

## Usage from C/C++

```c
#include <axon/lance_bridge.h>
#include <arrow/c/bridge.h>

// Create schema (using Arrow C++)
auto schema = arrow::schema({
    arrow::field("id", arrow::int64()),
    arrow::field("value", arrow::float64())
});

// Export schema to C format
struct ArrowSchema c_schema;
arrow::ExportSchema(*schema, &c_schema);

// Create or open dataset
int64_t handle = axon_lance_create_dataset("/path/to/data.lance", &c_schema);
if (handle <= 0) {
    const char* error = axon_lance_get_last_error();
    // Handle error
}

// Create and export a RecordBatch
auto batch = arrow::RecordBatch::Make(schema, num_rows, arrays);
struct ArrowArray c_array;
struct ArrowSchema c_batch_schema;
arrow::ExportRecordBatch(*batch, &c_array, &c_batch_schema);

// Write batch
int result = axon_lance_write_batch(handle, &c_array, &c_batch_schema);
if (result != AXON_LANCE_SUCCESS) {
    const char* error = axon_lance_get_last_error();
    // Handle error
}

// Close dataset
axon_lance_close_dataset(handle);
```

## API Reference

### Functions

| Function | Description |
|----------|-------------|
| `axon_lance_create_dataset(path, schema)` | Create/open dataset, returns handle |
| `axon_lance_write_batch(handle, array, schema)` | Write a RecordBatch |
| `axon_lance_close_dataset(handle)` | Close dataset and release resources |
| `axon_lance_get_last_error()` | Get last error message |

### Legacy Functions (Deprecated)

| Function | Replacement |
|----------|-------------|
| `create_or_open_dataset` | `axon_lance_create_dataset` |
| `write_batch` | `axon_lance_write_batch` |
| `close_dataset` | `axon_lance_close_dataset` |
| `lance_get_last_error` | `axon_lance_get_last_error` |

### Error Codes

| Code | Value | Description |
|------|-------|-------------|
| `AXON_LANCE_SUCCESS` | 0 | Operation successful |
| `AXON_LANCE_ERROR_INVALID_PATH` | -1 | Invalid file path |
| `AXON_LANCE_ERROR_INVALID_SCHEMA` | -2 | Invalid Arrow schema |
| `AXON_LANCE_ERROR_DATASET` | -3 | Dataset operation failed |
| `AXON_LANCE_ERROR_ARROW` | -4 | Arrow conversion failed |
| `AXON_LANCE_ERROR_IO` | -5 | IO error |
| `AXON_LANCE_ERROR_INVALID_HANDLE` | -6 | Invalid dataset handle |

## Building

```bash
# Build the FFI library and generate C header
cd c
cargo build --release

# Outputs:
# - target/release/liblance_writer_bridge.so (Linux)
# - target/release/liblance_writer_bridge.dylib (macOS)
# - include/axon/lance_bridge.h (C header)
```

## Linking

### CMake

```cmake
# Find the library
find_library(LANCE_BRIDGE lance_writer_bridge PATHS ${AXON_DIR}/c/target/release)

# Include headers
include_directories(${AXON_DIR}/c/include)

# Link
target_link_libraries(your_target ${LANCE_BRIDGE})
```

### pkg-config

```bash
gcc -I/path/to/c/include myapp.c -L/path/to/c/target/release -llance_writer_bridge
```

## Dependencies

This crate depends on:
- `axon-lance` (path = "../rust") - Core library
- `arrow` v54 - For FFI types

## See Also

- [rust/README.md](../rust/README.md) - Core Rust library
- [Main README](../README.md) - Project overview

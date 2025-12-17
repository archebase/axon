# Axon Lance - Core Rust Library

Core Rust library for writing Arrow data to Lance format.

## Overview

This crate provides the core functionality for writing data to Lance datasets:

- Create and open Lance datasets
- Write Arrow RecordBatches efficiently  
- Shared Tokio runtime for optimal performance
- Thread-safe handle-based API

## Structure

```
rust/
├── Cargo.toml          # Rust package configuration
├── README.md           # This file
└── src/
    ├── lib.rs          # Main library, public API
    ├── writer.rs       # LanceWriter implementation
    ├── error.rs        # Error types (LanceError)
    └── runtime.rs      # Shared Tokio runtime
```

## Usage

### Direct Rust Usage

```rust
use axon_lance::{LanceWriter, Schema, Field, DataType, RecordBatch};
use arrow_array::Int64Array;
use std::sync::Arc;

// Create schema
let schema = Arc::new(Schema::new(vec![
    Field::new("id", DataType::Int64, false),
    Field::new("value", DataType::Int64, false),
]));

// Create or open dataset
let handle = LanceWriter::create_or_open("/path/to/dataset.lance", schema.clone())?;

// Create a batch
let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
let values = Arc::new(Int64Array::from(vec![10, 20, 30]));
let batch = RecordBatch::try_new(schema, vec![ids, values])?;

// Write batch
LanceWriter::write_batch(handle, batch)?;

// Close dataset
LanceWriter::close(handle)?;
```

### From C/C++ via FFI

For C/C++ integration, use the `axon-lance-ffi` crate in the `c/` directory which provides a C-compatible interface.

## Architecture

```
┌─────────────────────────────────────────────────┐
│           Rust Application / C++ via FFI        │
└─────────────────────────────────────────────────┘
                     │
                     │ uses
                     ▼
┌─────────────────────────────────────────────────┐
│            axon-lance (this crate)              │
│         LanceWriter, error, runtime             │
└─────────────────────────────────────────────────┘
                     │
                     │ uses
                     ▼
┌─────────────────────────────────────────────────┐
│              Lance Library (v1.0)               │
└─────────────────────────────────────────────────┘
```

## API Reference

### `LanceWriter`

Main struct for dataset operations.

- `create_or_open(path, schema) -> Result<DatasetHandle>` - Create or open a dataset
- `write_batch(handle, batch) -> Result<()>` - Write a RecordBatch
- `close(handle) -> Result<()>` - Close and release resources
- `is_valid_handle(handle) -> bool` - Check if handle is valid
- `get_schema(handle) -> Result<Arc<Schema>>` - Get dataset schema

### Error Types

`LanceError` enum with variants:
- `InvalidPath` - Invalid file path
- `InvalidSchema` - Invalid Arrow schema  
- `DatasetError` - Lance dataset operation failed
- `ArrowError` - Arrow conversion failed
- `IoError` - IO operation failed
- `InvalidHandle` - Invalid dataset handle
- `RuntimeError` - Tokio runtime error

## Performance

The shared Tokio runtime approach significantly improves performance:

- **Before**: Creating new runtime for each operation (~1-2ms overhead)
- **After**: Reusing shared runtime (<0.1ms overhead)

## Dependencies

- **lance**: 1.0 (Lance columnar format)
- **arrow**: 54.x (Arrow C Data Interface)
- **tokio**: 1.x (async runtime)
- **once_cell**: 1.19 (lazy initialization)

## Building

```bash
cargo build --release
```

## Testing

```bash
cargo test
```

## Documentation

```bash
cargo doc --open
```

## See Also

- [c/README.md](../c/README.md) - C FFI layer documentation
- [Main README](../README.md) - Project overview

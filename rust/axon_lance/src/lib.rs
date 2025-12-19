//! Axon Lance - Core Rust library for writing data to Lance format
//!
//! This crate provides the core functionality for writing Arrow data to Lance datasets.
//! It is designed to be used directly from Rust applications or through the C FFI layer.
//!
//! # Features
//!
//! - Create and open Lance datasets
//! - Write Arrow RecordBatches efficiently
//! - Shared Tokio runtime for optimal performance
//! - Thread-safe handle-based API
//!
//! # Example
//!
//! ```ignore
//! use axon_lance::LanceWriter;
//! use arrow_schema::{Schema, Field, DataType};
//! use arrow_array::{RecordBatch, Int64Array};
//! use std::sync::Arc;
//!
//! // Create schema
//! let schema = Arc::new(Schema::new(vec![
//!     Field::new("id", DataType::Int64, false),
//!     Field::new("value", DataType::Int64, false),
//! ]));
//!
//! // Create or open dataset
//! let handle = LanceWriter::create_or_open("/path/to/dataset.lance", schema.clone())?;
//!
//! // Create a batch
//! let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
//! let values = Arc::new(Int64Array::from(vec![10, 20, 30]));
//! let batch = RecordBatch::try_new(schema, vec![ids, values])?;
//!
//! // Write batch
//! LanceWriter::write_batch(handle, batch)?;
//!
//! // Close dataset
//! LanceWriter::close(handle)?;
//! # Ok::<(), axon_lance::LanceError>(())
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────┐
//! │           Your Rust Application                 │
//! └─────────────────────────────────────────────────┘
//!                      │
//!                      │ uses
//!                      ▼
//! ┌─────────────────────────────────────────────────┐
//! │            axon-lance (this crate)              │
//! │         LanceWriter, error, runtime             │
//! └─────────────────────────────────────────────────┘
//!                      │
//!                      │ uses
//!                      ▼
//! ┌─────────────────────────────────────────────────┐
//! │              Lance Library (v1.0)               │
//! └─────────────────────────────────────────────────┘
//! ```

// Declare modules
pub mod error;
pub mod runtime;
mod writer;

// Re-exports for convenience
pub use error::{LanceError, Result};
pub use writer::{DatasetHandle, LanceWriter};
pub use runtime::{get_runtime, block_on, wait_for_pending_writes, pending_write_count};

// Re-export commonly used Arrow types
pub use arrow_schema::{Schema, Field, DataType};
pub use arrow_array::RecordBatch;

//! Error types for Axon Lance operations
//!
//! This module provides error types for Lance dataset operations.

use thiserror::Error;

/// Error types for Lance operations
#[derive(Error, Debug)]
pub enum LanceError {
    /// Invalid path provided
    #[error("Invalid path: {0}")]
    InvalidPath(String),

    /// Invalid schema provided
    #[error("Invalid schema: {0}")]
    InvalidSchema(String),

    /// Dataset operation error
    #[error("Dataset error: {0}")]
    DatasetError(String),

    /// Arrow conversion error
    #[error("Arrow error: {0}")]
    ArrowError(String),

    /// IO error
    #[error("IO error: {0}")]
    IoError(String),

    /// Invalid handle
    #[error("Invalid handle: {0}")]
    InvalidHandle(String),

    /// Runtime error
    #[error("Runtime error: {0}")]
    RuntimeError(String),
}

impl From<lance::Error> for LanceError {
    fn from(err: lance::Error) -> Self {
        LanceError::DatasetError(err.to_string())
    }
}

impl From<arrow::error::ArrowError> for LanceError {
    fn from(err: arrow::error::ArrowError) -> Self {
        LanceError::ArrowError(err.to_string())
    }
}

impl From<std::io::Error> for LanceError {
    fn from(err: std::io::Error) -> Self {
        LanceError::IoError(err.to_string())
    }
}

/// Result type for Lance operations
pub type Result<T> = std::result::Result<T, LanceError>;

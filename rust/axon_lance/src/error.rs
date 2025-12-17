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

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Error as IoError, ErrorKind};

    // ==========================================================================
    // Error Creation Tests
    // ==========================================================================

    #[test]
    fn test_invalid_path_error() {
        let error = LanceError::InvalidPath("path/does/not/exist".to_string());
        assert!(error.to_string().contains("Invalid path"));
        assert!(error.to_string().contains("path/does/not/exist"));
    }

    #[test]
    fn test_invalid_schema_error() {
        let error = LanceError::InvalidSchema("missing required field".to_string());
        assert!(error.to_string().contains("Invalid schema"));
        assert!(error.to_string().contains("missing required field"));
    }

    #[test]
    fn test_dataset_error() {
        let error = LanceError::DatasetError("failed to open dataset".to_string());
        assert!(error.to_string().contains("Dataset error"));
        assert!(error.to_string().contains("failed to open dataset"));
    }

    #[test]
    fn test_arrow_error() {
        let error = LanceError::ArrowError("invalid column type".to_string());
        assert!(error.to_string().contains("Arrow error"));
        assert!(error.to_string().contains("invalid column type"));
    }

    #[test]
    fn test_io_error() {
        let error = LanceError::IoError("permission denied".to_string());
        assert!(error.to_string().contains("IO error"));
        assert!(error.to_string().contains("permission denied"));
    }

    #[test]
    fn test_invalid_handle_error() {
        let error = LanceError::InvalidHandle("handle 42 not found".to_string());
        assert!(error.to_string().contains("Invalid handle"));
        assert!(error.to_string().contains("handle 42 not found"));
    }

    #[test]
    fn test_runtime_error() {
        let error = LanceError::RuntimeError("tokio runtime panic".to_string());
        assert!(error.to_string().contains("Runtime error"));
        assert!(error.to_string().contains("tokio runtime panic"));
    }

    // ==========================================================================
    // Error Conversion Tests
    // ==========================================================================

    #[test]
    fn test_from_io_error() {
        let io_error = IoError::new(ErrorKind::NotFound, "file not found");
        let lance_error: LanceError = io_error.into();
        
        match lance_error {
            LanceError::IoError(msg) => {
                assert!(msg.contains("file not found"));
            }
            _ => panic!("Expected IoError variant"),
        }
    }

    #[test]
    fn test_from_io_error_permission_denied() {
        let io_error = IoError::new(ErrorKind::PermissionDenied, "access denied");
        let lance_error: LanceError = io_error.into();
        
        match lance_error {
            LanceError::IoError(msg) => {
                assert!(msg.contains("access denied"));
            }
            _ => panic!("Expected IoError variant"),
        }
    }

    #[test]
    fn test_from_arrow_error() {
        let arrow_error = arrow::error::ArrowError::InvalidArgumentError(
            "invalid argument".to_string()
        );
        let lance_error: LanceError = arrow_error.into();
        
        match lance_error {
            LanceError::ArrowError(msg) => {
                assert!(msg.contains("invalid argument"));
            }
            _ => panic!("Expected ArrowError variant"),
        }
    }

    #[test]
    fn test_from_arrow_schema_error() {
        let arrow_error = arrow::error::ArrowError::SchemaError(
            "schema mismatch".to_string()
        );
        let lance_error: LanceError = arrow_error.into();
        
        match lance_error {
            LanceError::ArrowError(msg) => {
                assert!(msg.contains("schema mismatch"));
            }
            _ => panic!("Expected ArrowError variant"),
        }
    }

    // ==========================================================================
    // Debug and Display Tests
    // ==========================================================================

    #[test]
    fn test_error_debug_format() {
        let error = LanceError::InvalidPath("test/path".to_string());
        let debug_str = format!("{:?}", error);
        
        assert!(debug_str.contains("InvalidPath"));
        assert!(debug_str.contains("test/path"));
    }

    #[test]
    fn test_error_display_format() {
        let error = LanceError::DatasetError("test error message".to_string());
        let display_str = format!("{}", error);
        
        assert_eq!(display_str, "Dataset error: test error message");
    }

    #[test]
    fn test_all_variants_display() {
        let variants = vec![
            LanceError::InvalidPath("path".to_string()),
            LanceError::InvalidSchema("schema".to_string()),
            LanceError::DatasetError("dataset".to_string()),
            LanceError::ArrowError("arrow".to_string()),
            LanceError::IoError("io".to_string()),
            LanceError::InvalidHandle("handle".to_string()),
            LanceError::RuntimeError("runtime".to_string()),
        ];
        
        for error in variants {
            let display = format!("{}", error);
            assert!(!display.is_empty());
        }
    }

    // ==========================================================================
    // Result Type Tests
    // ==========================================================================

    #[test]
    fn test_result_ok() {
        let result: Result<i32> = Ok(42);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42);
    }

    #[test]
    fn test_result_err() {
        let result: Result<i32> = Err(LanceError::InvalidHandle("test".to_string()));
        assert!(result.is_err());
    }

    #[test]
    fn test_result_map() {
        let result: Result<i32> = Ok(10);
        let mapped = result.map(|x| x * 2);
        assert_eq!(mapped.unwrap(), 20);
    }

    #[test]
    fn test_result_and_then() {
        let result: Result<i32> = Ok(10);
        let chained = result.and_then(|x| {
            if x > 5 {
                Ok(x * 2)
            } else {
                Err(LanceError::InvalidPath("too small".to_string()))
            }
        });
        assert_eq!(chained.unwrap(), 20);
    }

    #[test]
    fn test_result_unwrap_or() {
        let ok_result: Result<i32> = Ok(42);
        let err_result: Result<i32> = Err(LanceError::RuntimeError("error".to_string()));
        
        assert_eq!(ok_result.unwrap_or(0), 42);
        assert_eq!(err_result.unwrap_or(0), 0);
    }

    // ==========================================================================
    // Error Matching Tests
    // ==========================================================================

    #[test]
    fn test_error_pattern_matching() {
        let error = LanceError::InvalidHandle("handle 123".to_string());
        
        match error {
            LanceError::InvalidHandle(msg) => {
                assert!(msg.contains("123"));
            }
            _ => panic!("Wrong error variant"),
        }
    }

    #[test]
    fn test_error_is_specific_type() {
        let error = LanceError::DatasetError("test".to_string());
        
        let is_dataset_error = matches!(error, LanceError::DatasetError(_));
        assert!(is_dataset_error);
        
        let is_io_error = matches!(error, LanceError::IoError(_));
        assert!(!is_io_error);
    }
}

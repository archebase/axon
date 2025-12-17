//! Lance dataset writer
//!
//! This module provides the core functionality for writing data to Lance format.

use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex, atomic::{AtomicI64, Ordering}};

use arrow_array::RecordBatch;
use arrow_schema::Schema;
use arrow::record_batch::RecordBatchIterator;
use lance::dataset::{Dataset, WriteMode, WriteParams};

use crate::error::{LanceError, Result};
use crate::runtime::block_on;

/// Handle type for dataset references
pub type DatasetHandle = i64;

/// Internal state for an open dataset
struct DatasetState {
    dataset: Arc<tokio::sync::Mutex<Dataset>>,
    #[allow(dead_code)]
    path: String,
    schema: Arc<Schema>,
}

/// Global handle counter
static NEXT_HANDLE: AtomicI64 = AtomicI64::new(1);

lazy_static::lazy_static! {
    /// Global dataset registry
    static ref DATASETS: Arc<Mutex<HashMap<DatasetHandle, DatasetState>>> = 
        Arc::new(Mutex::new(HashMap::new()));
}

fn get_next_handle() -> DatasetHandle {
    NEXT_HANDLE.fetch_add(1, Ordering::SeqCst)
}

/// Lance dataset writer
///
/// Provides methods for creating, opening, and writing to Lance datasets.
///
/// # Example
///
/// ```ignore
/// use axon_lance::LanceWriter;
/// use arrow_schema::{Schema, Field, DataType};
/// use std::sync::Arc;
///
/// let schema = Arc::new(Schema::new(vec![
///     Field::new("id", DataType::Int64, false),
/// ]));
///
/// let handle = LanceWriter::create_or_open("/path/to/dataset.lance", schema)?;
/// // ... write batches ...
/// LanceWriter::close(handle)?;
/// ```
pub struct LanceWriter;

impl LanceWriter {
    /// Create or open a Lance dataset
    ///
    /// Opens an existing dataset at the given path, or creates a new one if it doesn't exist.
    ///
    /// # Arguments
    /// * `path` - Path to the dataset
    /// * `schema` - Arrow schema for the dataset
    ///
    /// # Returns
    /// * `Ok(handle)` - A handle to use for subsequent operations
    /// * `Err(LanceError)` - If the operation fails
    pub fn create_or_open(path: &str, schema: Arc<Schema>) -> Result<DatasetHandle> {
        let path_buf = Path::new(path);
        
        let dataset = if path_buf.exists() {
            // Open existing dataset
            block_on(async {
                Dataset::open(path)
                    .await
                    .map_err(|e| LanceError::DatasetError(format!("Failed to open dataset: {}", e)))
            })?
        } else {
            // Create new dataset with empty batch to establish schema
            let empty_batch = RecordBatch::new_empty(schema.clone());
            
            block_on(async {
                let params = WriteParams {
                    mode: WriteMode::Create,
                    ..Default::default()
                };
                
                let batch_iter = RecordBatchIterator::new(
                    std::iter::once(Ok(empty_batch)),
                    schema.clone(),
                );
                
                Dataset::write(batch_iter, path, Some(params))
                    .await
                    .map_err(|e| LanceError::DatasetError(format!("Failed to create dataset: {}", e)))
            })?
        };
        
        let handle = get_next_handle();
        let state = DatasetState {
            dataset: Arc::new(tokio::sync::Mutex::new(dataset)),
            path: path.to_string(),
            schema,
        };
        
        DATASETS.lock().unwrap().insert(handle, state);
        Ok(handle)
    }

    /// Write a RecordBatch to the dataset
    ///
    /// Appends the given batch to the dataset.
    ///
    /// # Arguments
    /// * `handle` - Dataset handle from `create_or_open`
    /// * `batch` - The RecordBatch to write
    ///
    /// # Returns
    /// * `Ok(())` - If the write succeeds
    /// * `Err(LanceError)` - If the operation fails
    pub fn write_batch(handle: DatasetHandle, batch: RecordBatch) -> Result<()> {
        let datasets = DATASETS.lock().unwrap();
        let state = datasets.get(&handle)
            .ok_or_else(|| LanceError::InvalidHandle(format!("Invalid dataset handle: {}", handle)))?;
        
        let dataset = Arc::clone(&state.dataset);
        let schema = Arc::clone(&state.schema);
        
        block_on(async {
            let params = WriteParams {
                mode: WriteMode::Append,
                ..Default::default()
            };
            
            let batch_iter = RecordBatchIterator::new(
                std::iter::once(Ok(batch)),
                schema,
            );
            
            let mut dataset_guard = dataset.lock().await;
            dataset_guard
                .append(batch_iter, Some(params))
                .await
                .map_err(|e| LanceError::DatasetError(format!("Failed to append batch: {}", e)))
        })?;
        
        Ok(())
    }

    /// Close a dataset and release resources
    ///
    /// # Arguments
    /// * `handle` - Dataset handle to close
    ///
    /// # Returns
    /// * `Ok(())` - If the close succeeds
    /// * `Err(LanceError)` - If the handle is invalid
    pub fn close(handle: DatasetHandle) -> Result<()> {
        let mut datasets = DATASETS.lock().unwrap();
        datasets.remove(&handle)
            .ok_or_else(|| LanceError::InvalidHandle(format!("Invalid dataset handle: {}", handle)))?;
        Ok(())
    }

    /// Check if a handle is valid
    pub fn is_valid_handle(handle: DatasetHandle) -> bool {
        DATASETS.lock().unwrap().contains_key(&handle)
    }

    /// Get the schema for a dataset
    pub fn get_schema(handle: DatasetHandle) -> Result<Arc<Schema>> {
        let datasets = DATASETS.lock().unwrap();
        let state = datasets.get(&handle)
            .ok_or_else(|| LanceError::InvalidHandle(format!("Invalid dataset handle: {}", handle)))?;
        Ok(Arc::clone(&state.schema))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow_array::Int64Array;
    use arrow_schema::{DataType, Field};
    use tempfile::TempDir;

    #[test]
    fn test_create_dataset() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_dataset.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema).unwrap();
        assert!(handle > 0);
        assert!(LanceWriter::is_valid_handle(handle));
        
        LanceWriter::close(handle).unwrap();
        assert!(!LanceWriter::is_valid_handle(handle));
    }

    #[test]
    fn test_write_batch() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_write.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Create and write a batch
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let batch = RecordBatch::try_new(schema, vec![ids]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_invalid_handle() {
        let result = LanceWriter::close(9999);
        assert!(result.is_err());
    }
}

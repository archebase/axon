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
    use arrow_array::{Int64Array, StringArray, Float64Array};
    use arrow_schema::{DataType, Field};
    use tempfile::TempDir;

    // ==========================================================================
    // Basic Creation Tests
    // ==========================================================================

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
    fn test_create_dataset_multi_column() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_multi_column.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("name", DataType::Utf8, true),
            Field::new("value", DataType::Float64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema).unwrap();
        assert!(LanceWriter::is_valid_handle(handle));
        
        LanceWriter::close(handle).unwrap();
    }

    // ==========================================================================
    // Write Tests
    // ==========================================================================

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
    fn test_write_multiple_batches() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_multi_batch.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Write multiple batches
        for i in 0..5 {
            let ids = Arc::new(Int64Array::from(vec![i * 10, i * 10 + 1, i * 10 + 2]));
            let batch = RecordBatch::try_new(schema.clone(), vec![ids]).unwrap();
            LanceWriter::write_batch(handle, batch).unwrap();
        }
        
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_write_multi_column_batch() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_multi_col_write.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("name", DataType::Utf8, true),
            Field::new("score", DataType::Float64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let names = Arc::new(StringArray::from(vec![Some("Alice"), None, Some("Charlie")]));
        let scores = Arc::new(Float64Array::from(vec![95.5, 87.3, 91.0]));
        
        let batch = RecordBatch::try_new(
            schema.clone(), 
            vec![ids, names, scores]
        ).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_write_empty_batch() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_empty_batch.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Write an empty batch
        let empty_batch = RecordBatch::new_empty(schema.clone());
        let result = LanceWriter::write_batch(handle, empty_batch);
        // Empty batch write should succeed (it's valid, just no data)
        assert!(result.is_ok());
        
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_write_large_batch() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_large_batch.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("value", DataType::Float64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Create a large batch (10000 rows)
        let ids: Vec<i64> = (0..10000).collect();
        let values: Vec<f64> = (0..10000).map(|i| i as f64 * 0.1).collect();
        
        let id_array = Arc::new(Int64Array::from(ids));
        let value_array = Arc::new(Float64Array::from(values));
        
        let batch = RecordBatch::try_new(schema.clone(), vec![id_array, value_array]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    // ==========================================================================
    // Reopen Tests
    // ==========================================================================

    #[test]
    fn test_reopen_existing_dataset() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_reopen.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        // Create and write initial data
        let handle1 = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        let ids1 = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let batch1 = RecordBatch::try_new(schema.clone(), vec![ids1]).unwrap();
        LanceWriter::write_batch(handle1, batch1).unwrap();
        LanceWriter::close(handle1).unwrap();
        
        // Reopen and write more data
        let handle2 = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        assert!(LanceWriter::is_valid_handle(handle2));
        
        let ids2 = Arc::new(Int64Array::from(vec![4, 5, 6]));
        let batch2 = RecordBatch::try_new(schema.clone(), vec![ids2]).unwrap();
        LanceWriter::write_batch(handle2, batch2).unwrap();
        
        LanceWriter::close(handle2).unwrap();
    }

    #[test]
    fn test_multiple_handles() {
        let temp_dir = TempDir::new().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        // Create multiple datasets simultaneously
        let path1 = temp_dir.path().join("dataset1.lance");
        let path2 = temp_dir.path().join("dataset2.lance");
        
        let handle1 = LanceWriter::create_or_open(path1.to_str().unwrap(), schema.clone()).unwrap();
        let handle2 = LanceWriter::create_or_open(path2.to_str().unwrap(), schema.clone()).unwrap();
        
        // Handles should be different
        assert_ne!(handle1, handle2);
        assert!(LanceWriter::is_valid_handle(handle1));
        assert!(LanceWriter::is_valid_handle(handle2));
        
        // Write to both
        let ids1 = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let batch1 = RecordBatch::try_new(schema.clone(), vec![ids1]).unwrap();
        LanceWriter::write_batch(handle1, batch1).unwrap();
        
        let ids2 = Arc::new(Int64Array::from(vec![10, 20, 30]));
        let batch2 = RecordBatch::try_new(schema.clone(), vec![ids2]).unwrap();
        LanceWriter::write_batch(handle2, batch2).unwrap();
        
        // Close both
        LanceWriter::close(handle1).unwrap();
        LanceWriter::close(handle2).unwrap();
    }

    // ==========================================================================
    // Schema Tests
    // ==========================================================================

    #[test]
    fn test_get_schema() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_get_schema.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("name", DataType::Utf8, true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        let retrieved_schema = LanceWriter::get_schema(handle).unwrap();
        
        assert_eq!(retrieved_schema.fields().len(), 2);
        assert_eq!(retrieved_schema.field(0).name(), "id");
        assert_eq!(retrieved_schema.field(1).name(), "name");
        
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_get_schema_invalid_handle() {
        let result = LanceWriter::get_schema(99999);
        assert!(result.is_err());
        
        match result {
            Err(LanceError::InvalidHandle(_)) => {}
            _ => panic!("Expected InvalidHandle error"),
        }
    }

    // ==========================================================================
    // Error Handling Tests
    // ==========================================================================

    #[test]
    fn test_invalid_handle() {
        let result = LanceWriter::close(9999);
        assert!(result.is_err());
    }

    #[test]
    fn test_write_to_invalid_handle() {
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let batch = RecordBatch::try_new(schema, vec![ids]).unwrap();
        
        let result = LanceWriter::write_batch(99999, batch);
        assert!(result.is_err());
        
        match result {
            Err(LanceError::InvalidHandle(_)) => {}
            _ => panic!("Expected InvalidHandle error"),
        }
    }

    #[test]
    fn test_close_already_closed_handle() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_double_close.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema).unwrap();
        
        // First close succeeds
        LanceWriter::close(handle).unwrap();
        
        // Second close should fail (handle no longer valid)
        let result = LanceWriter::close(handle);
        assert!(result.is_err());
    }

    #[test]
    fn test_is_valid_handle() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_valid_handle.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        // Invalid before creation
        assert!(!LanceWriter::is_valid_handle(99999));
        
        let handle = LanceWriter::create_or_open(path_str, schema).unwrap();
        
        // Valid after creation
        assert!(LanceWriter::is_valid_handle(handle));
        
        LanceWriter::close(handle).unwrap();
        
        // Invalid after close
        assert!(!LanceWriter::is_valid_handle(handle));
    }

    // ==========================================================================
    // Handle Counter Tests
    // ==========================================================================

    #[test]
    fn test_handle_uniqueness() {
        let temp_dir = TempDir::new().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let mut handles = Vec::new();
        
        // Create multiple handles
        for i in 0..10 {
            let path = temp_dir.path().join(format!("dataset_{}.lance", i));
            let handle = LanceWriter::create_or_open(path.to_str().unwrap(), schema.clone()).unwrap();
            
            // Each handle should be unique
            assert!(!handles.contains(&handle));
            handles.push(handle);
        }
        
        // Clean up
        for handle in handles {
            LanceWriter::close(handle).unwrap();
        }
    }

    // ==========================================================================
    // Thread Safety Tests
    // ==========================================================================

    #[test]
    fn test_concurrent_writes_to_different_datasets() {
        use std::thread;
        
        let temp_dir = TempDir::new().unwrap();
        let temp_path = temp_dir.path().to_path_buf();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("thread_id", DataType::Int64, false),
        ]));
        
        let handles: Vec<_> = (0..4).map(|i| {
            let path = temp_path.join(format!("concurrent_{}.lance", i));
            let schema_clone = schema.clone();
            thread::spawn(move || {
                let handle = LanceWriter::create_or_open(path.to_str().unwrap(), schema_clone.clone()).unwrap();
                
                // Write multiple batches
                for batch_num in 0..5 {
                    let ids = Arc::new(Int64Array::from(vec![batch_num * 10, batch_num * 10 + 1]));
                    let thread_ids = Arc::new(Int64Array::from(vec![i as i64, i as i64]));
                    let batch = RecordBatch::try_new(schema_clone.clone(), vec![ids, thread_ids]).unwrap();
                    LanceWriter::write_batch(handle, batch).unwrap();
                }
                
                handle
            })
        }).collect();
        
        // Wait for all threads and collect handles
        let dataset_handles: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        
        // Verify all handles are valid
        for handle in &dataset_handles {
            assert!(LanceWriter::is_valid_handle(*handle));
        }
        
        // Close all
        for handle in dataset_handles {
            LanceWriter::close(handle).unwrap();
        }
    }

    #[test]
    fn test_concurrent_handle_creation() {
        use std::thread;
        use std::sync::atomic::{AtomicUsize, Ordering};
        
        let temp_dir = TempDir::new().unwrap();
        let temp_path = temp_dir.path().to_path_buf();
        let counter = Arc::new(AtomicUsize::new(0));
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handles: Vec<_> = (0..8).map(|_| {
            let path = temp_path.clone();
            let schema_clone = schema.clone();
            let counter_clone = counter.clone();
            
            thread::spawn(move || {
                let idx = counter_clone.fetch_add(1, Ordering::SeqCst);
                let dataset_path = path.join(format!("thread_test_{}.lance", idx));
                let handle = LanceWriter::create_or_open(dataset_path.to_str().unwrap(), schema_clone).unwrap();
                handle
            })
        }).collect();
        
        let dataset_handles: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        
        // All handles should be unique
        let unique_handles: std::collections::HashSet<_> = dataset_handles.iter().collect();
        assert_eq!(unique_handles.len(), dataset_handles.len());
        
        // Clean up
        for handle in dataset_handles {
            LanceWriter::close(handle).unwrap();
        }
    }

    #[test]
    fn test_sequential_write_then_read_handle() {
        use std::thread;
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("seq_write_read.lance");
        let path_str = dataset_path.to_str().unwrap().to_string();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        // Create dataset in main thread
        let handle = LanceWriter::create_or_open(&path_str, schema.clone()).unwrap();
        
        // Write from multiple threads sequentially (using the same handle)
        // Note: This tests that the handle registry is thread-safe
        let handle_copy = handle;
        let schema_clone = schema.clone();
        
        let writer_thread = thread::spawn(move || {
            let ids = Arc::new(Int64Array::from(vec![100, 200, 300]));
            let batch = RecordBatch::try_new(schema_clone, vec![ids]).unwrap();
            LanceWriter::write_batch(handle_copy, batch)
        });
        
        let result = writer_thread.join().unwrap();
        assert!(result.is_ok());
        
        // Verify handle still valid in main thread
        assert!(LanceWriter::is_valid_handle(handle));
        
        LanceWriter::close(handle).unwrap();
    }

    // ==========================================================================
    // Nested Data Type Tests
    // ==========================================================================

    #[test]
    fn test_list_column() {
        use arrow_array::builder::{ListBuilder, Int64Builder};
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_list.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        // Schema with a list column
        let list_field = Field::new("item", DataType::Int64, true);
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("values", DataType::List(Arc::new(list_field)), true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Build list array
        let mut list_builder = ListBuilder::new(Int64Builder::new());
        
        // First row: [1, 2, 3]
        list_builder.values().append_value(1);
        list_builder.values().append_value(2);
        list_builder.values().append_value(3);
        list_builder.append(true);
        
        // Second row: [10, 20]
        list_builder.values().append_value(10);
        list_builder.values().append_value(20);
        list_builder.append(true);
        
        // Third row: null
        list_builder.append(false);
        
        let list_array = list_builder.finish();
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![ids, Arc::new(list_array)],
        ).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_struct_column() {
        use arrow_array::StructArray;
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_struct.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        // Schema with a struct column
        let struct_fields = vec![
            Field::new("x", DataType::Float64, false),
            Field::new("y", DataType::Float64, false),
        ];
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("point", DataType::Struct(struct_fields.clone().into()), true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Build struct array
        let x_array = Arc::new(Float64Array::from(vec![1.0, 2.0, 3.0]));
        let y_array = Arc::new(Float64Array::from(vec![4.0, 5.0, 6.0]));
        
        let struct_array = StructArray::try_new(
            struct_fields.into(),
            vec![x_array, y_array],
            None,
        ).unwrap();
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![ids, Arc::new(struct_array)],
        ).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_nullable_columns() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_nullable.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("optional_name", DataType::Utf8, true),
            Field::new("optional_value", DataType::Float64, true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Create arrays with null values
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3, 4]));
        let names = Arc::new(StringArray::from(vec![
            Some("Alice"),
            None,
            Some("Charlie"),
            None,
        ]));
        let values = Arc::new(Float64Array::from(vec![
            Some(1.5),
            Some(2.5),
            None,
            None,
        ]));
        
        let batch = RecordBatch::try_new(schema.clone(), vec![ids, names, values]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_binary_column() {
        use arrow_array::BinaryArray;
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_binary.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("data", DataType::Binary, true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let binary_data = Arc::new(BinaryArray::from(vec![
            Some(b"hello".as_slice()),
            Some(b"\x00\x01\x02\x03".as_slice()),
            None,
        ]));
        
        let batch = RecordBatch::try_new(schema.clone(), vec![ids, binary_data]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_boolean_column() {
        use arrow_array::BooleanArray;
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_boolean.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("flag", DataType::Boolean, true),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3, 4]));
        let flags = Arc::new(BooleanArray::from(vec![
            Some(true),
            Some(false),
            None,
            Some(true),
        ]));
        
        let batch = RecordBatch::try_new(schema.clone(), vec![ids, flags]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_timestamp_column() {
        use arrow_array::TimestampNanosecondArray;
        use arrow_schema::TimeUnit;
        
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_timestamp.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("timestamp", DataType::Timestamp(TimeUnit::Nanosecond, None), false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        // Timestamps in nanoseconds
        let timestamps = Arc::new(TimestampNanosecondArray::from(vec![
            1_000_000_000_000_000_000i64,  // 1 second in ns
            2_000_000_000_000_000_000i64,
            3_000_000_000_000_000_000i64,
        ]));
        
        let batch = RecordBatch::try_new(schema.clone(), vec![ids, timestamps]).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }

    // ==========================================================================
    // Stress Tests
    // ==========================================================================

    #[test]
    fn test_many_small_batches() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_many_batches.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Write 100 small batches
        for i in 0..100 {
            let ids = Arc::new(Int64Array::from(vec![i]));
            let batch = RecordBatch::try_new(schema.clone(), vec![ids]).unwrap();
            LanceWriter::write_batch(handle, batch).unwrap();
        }
        
        LanceWriter::close(handle).unwrap();
    }

    #[test]
    fn test_wide_schema() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_wide_schema.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        // Create schema with many columns
        let fields: Vec<Field> = (0..50)
            .map(|i| Field::new(format!("col_{}", i), DataType::Int64, false))
            .collect();
        let schema = Arc::new(Schema::new(fields));
        
        let handle = LanceWriter::create_or_open(path_str, schema.clone()).unwrap();
        
        // Create arrays for all columns
        let arrays: Vec<Arc<dyn arrow_array::Array>> = (0..50)
            .map(|i| Arc::new(Int64Array::from(vec![i as i64, i as i64 + 1, i as i64 + 2])) as Arc<dyn arrow_array::Array>)
            .collect();
        
        let batch = RecordBatch::try_new(schema.clone(), arrays).unwrap();
        
        LanceWriter::write_batch(handle, batch).unwrap();
        LanceWriter::close(handle).unwrap();
    }
}

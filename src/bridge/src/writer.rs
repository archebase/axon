use std::collections::HashMap;
use std::path::Path;
use std::sync::{Arc, Mutex};

use arrow::ffi::{FFI_ArrowArray, FFI_ArrowSchema};
use arrow::record_batch::{RecordBatch, RecordBatchIterator};
use arrow::datatypes::Schema;
use lance::dataset::{Dataset, WriteMode, WriteParams};

use crate::error::{LanceError, Result};

// Global dataset handle manager
type DatasetHandle = i64;
static mut NEXT_HANDLE: DatasetHandle = 1;

struct DatasetState {
    dataset: Arc<tokio::sync::Mutex<Dataset>>,
    #[allow(dead_code)]
    path: String,
    schema: Arc<Schema>,
}

lazy_static::lazy_static! {
    static ref DATASETS: Arc<Mutex<HashMap<DatasetHandle, DatasetState>>> = 
        Arc::new(Mutex::new(HashMap::new()));
}

fn get_next_handle() -> DatasetHandle {
    unsafe {
        let handle = NEXT_HANDLE;
        NEXT_HANDLE += 1;
        handle
    }
}

pub fn create_or_open_dataset_internal(
    path: &str,
    schema: Arc<Schema>,
) -> Result<DatasetHandle> {
    let path_buf = Path::new(path);
    
    // Check if dataset exists
    let dataset = if path_buf.exists() {
        // Open existing dataset
        let rt = tokio::runtime::Runtime::new()
            .map_err(|e| LanceError::IOError(format!("Failed to create runtime: {}", e)))?;
        
        rt.block_on(async {
            Dataset::open(path)
                .await
                .map_err(|e| LanceError::DatasetError(format!("Failed to open dataset: {}", e)))
        })?
    } else {
        // Create new dataset with empty batch to establish schema
        let rt = tokio::runtime::Runtime::new()
            .map_err(|e| LanceError::IOError(format!("Failed to create runtime: {}", e)))?;
        
        // Create empty batch with schema
        let empty_batch = RecordBatch::new_empty(schema.clone());
        
        rt.block_on(async {
            let params = WriteParams {
                mode: WriteMode::Create,
                ..Default::default()
            };
            
            // Convert Vec<RecordBatch> to RecordBatchIterator
            let batches = vec![empty_batch];
            let batch_iter = RecordBatchIterator::new(
                batches.into_iter().map(Ok),
                schema.clone(),
            );
            
            Dataset::write(
                batch_iter,
                path,
                Some(params),
            )
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

pub fn write_batch_internal(
    handle: DatasetHandle,
    array: *mut FFI_ArrowArray,
    schema: *mut FFI_ArrowSchema,
) -> Result<()> {
    let datasets = DATASETS.lock().unwrap();
    let state = datasets.get(&handle)
        .ok_or(LanceError::DatasetError("Invalid dataset handle".to_string()))?;
    
    // Import Arrow data from C interface (zero-copy)
    // In arrow 56, we need to use from_ffi which returns ArrayData, then convert to RecordBatch
    let record_batch = unsafe {
        // Read the FFI structures (the caller retains ownership)
        let ffi_array = std::ptr::read(array);
        let ffi_schema_ref = &*schema;
        let array_data = arrow::ffi::from_ffi(ffi_array, ffi_schema_ref)
            .map_err(|e| LanceError::ArrowError(format!("Failed to import Arrow data: {}", e)))?;
        
        // Convert ArrayData to RecordBatch
        // ArrayData from FFI should be a StructArray representing a RecordBatch
        use arrow::array::StructArray;
        let struct_array = StructArray::from(array_data);
        RecordBatch::from(struct_array)
    };
    
    // Write to dataset (append mode)
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| LanceError::IOError(format!("Failed to create runtime: {}", e)))?;
    
    let dataset = Arc::clone(&state.dataset);
    let schema = Arc::clone(&state.schema);
    rt.block_on(async {
        let params = WriteParams {
            mode: WriteMode::Append,
            ..Default::default()
        };
        
        // Convert RecordBatch to RecordBatchIterator
        let batch_iter = RecordBatchIterator::new(
            std::iter::once(Ok(record_batch)),
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

pub fn close_dataset_internal(handle: DatasetHandle) -> Result<()> {
    let mut datasets = DATASETS.lock().unwrap();
    datasets.remove(&handle)
        .ok_or(LanceError::DatasetError("Invalid dataset handle".to_string()))?;
    Ok(())
}

use std::ffi::CStr;
use std::os::raw::{c_char, c_int};
use std::sync::Arc;

use arrow::ffi::{FFI_ArrowArray, FFI_ArrowSchema};

mod error;
mod writer;

use error::{
    error_code, set_last_error, LanceError, LANCE_ERROR_INVALID_HANDLE,
    LANCE_ERROR_INVALID_SCHEMA, LANCE_SUCCESS,
};
use writer::{close_dataset_internal, create_or_open_dataset_internal, write_batch_internal};

/// Create or open a Lance dataset
/// 
/// # Arguments
/// * `path` - Path to the dataset (null-terminated C string)
/// * `schema_ptr` - Pointer to Arrow schema (FFI_ArrowSchema)
/// 
/// # Returns
/// Dataset handle (positive i64) on success, or 0 on error
#[no_mangle]
pub extern "C" fn create_or_open_dataset(
    path: *const c_char,
    schema_ptr: *mut FFI_ArrowSchema,
) -> i64 {
    if path.is_null() || schema_ptr.is_null() {
        set_last_error(&LanceError::InvalidPath);
        return 0;
    }

    unsafe {
        let path_str = match CStr::from_ptr(path).to_str() {
            Ok(s) => s,
            Err(_) => {
                set_last_error(&LanceError::InvalidPath);
                return 0;
            }
        };

        // Import schema from C interface
        // In arrow 56, we use Schema::try_from
        let schema = match arrow::datatypes::Schema::try_from(&*schema_ptr) {
            Ok(s) => s,
            Err(e) => {
                set_last_error(&LanceError::ArrowError(format!("Failed to import schema: {}", e)));
                return 0;
            }
        };

        match create_or_open_dataset_internal(path_str, Arc::new(schema)) {
            Ok(handle) => handle,
            Err(e) => {
                set_last_error(&e);
                0
            }
        }
    }
}

/// Write a RecordBatch to a Lance dataset
/// 
/// # Arguments
/// * `dataset_handle` - Handle returned by create_or_open_dataset
/// * `array_ptr` - Pointer to Arrow array (FFI_ArrowArray)
/// * `schema_ptr` - Pointer to Arrow schema (FFI_ArrowSchema)
/// 
/// # Returns
/// 0 on success, negative error code on failure
#[no_mangle]
pub extern "C" fn write_batch(
    dataset_handle: i64,
    array_ptr: *mut FFI_ArrowArray,
    schema_ptr: *mut FFI_ArrowSchema,
) -> c_int {
    if dataset_handle <= 0 {
        set_last_error(&LanceError::DatasetError("Invalid dataset handle".to_string()));
        return LANCE_ERROR_INVALID_HANDLE;
    }

    if array_ptr.is_null() || schema_ptr.is_null() {
        set_last_error(&LanceError::InvalidSchema);
        return LANCE_ERROR_INVALID_SCHEMA;
    }

    match write_batch_internal(dataset_handle, array_ptr, schema_ptr) {
        Ok(_) => LANCE_SUCCESS,
        Err(e) => {
            set_last_error(&e);
            error_code(&e)
        }
    }
}

/// Close a dataset and release resources
/// 
/// # Arguments
/// * `dataset_handle` - Handle returned by create_or_open_dataset
/// 
/// # Returns
/// 0 on success, negative error code on failure
#[no_mangle]
pub extern "C" fn close_dataset(dataset_handle: i64) -> c_int {
    if dataset_handle <= 0 {
        set_last_error(&LanceError::DatasetError("Invalid dataset handle".to_string()));
        return LANCE_ERROR_INVALID_HANDLE;
    }

    match close_dataset_internal(dataset_handle) {
        Ok(_) => LANCE_SUCCESS,
        Err(e) => {
            set_last_error(&e);
            error_code(&e)
        }
    }
}

// lance_get_last_error is already exported from error module

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::Int64Array;
    use arrow::datatypes::{DataType, Field, Schema};
    use arrow::record_batch::RecordBatch;
    use std::ffi::CString;
    use std::sync::Arc;
    use tempfile::TempDir;

    #[test]
    fn test_create_dataset() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_dataset.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        // Create schema
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
            Field::new("name", DataType::Utf8, false),
        ]));
        
        let c_path = CString::new(path_str).unwrap();
        let mut c_schema = FFI_ArrowSchema::empty();
        arrow::ffi::export_schema(&schema, &mut c_schema);
        
        let handle = create_or_open_dataset(c_path.as_ptr(), &mut c_schema);
        assert!(handle > 0, "Failed to create dataset");
        
        // Cleanup
        close_dataset(handle);
    }

    #[test]
    fn test_write_batch() {
        let temp_dir = TempDir::new().unwrap();
        let dataset_path = temp_dir.path().join("test_write.lance");
        let path_str = dataset_path.to_str().unwrap();
        
        // Create schema
        let schema = Arc::new(Schema::new(vec![
            Field::new("id", DataType::Int64, false),
        ]));
        
        let c_path = CString::new(path_str).unwrap();
        let mut c_schema = FFI_ArrowSchema::empty();
        arrow::ffi::export_schema(&schema, &mut c_schema);
        
        let handle = create_or_open_dataset(c_path.as_ptr(), &mut c_schema);
        assert!(handle > 0);
        
        // Create a batch
        let ids = Arc::new(Int64Array::from(vec![1, 2, 3]));
        let batch = RecordBatch::try_new(
            schema.clone(),
            vec![ids],
        ).unwrap();
        
        // Export batch
        let mut c_array = FFI_ArrowArray::empty();
        let mut c_schema_batch = FFI_ArrowSchema::empty();
        arrow::ffi::export_record_batch(&batch, &mut c_array, &mut c_schema_batch);
        
        // Write batch
        let result = write_batch(handle, &mut c_array, &mut c_schema_batch);
        assert_eq!(result, LANCE_SUCCESS);
        
        // Cleanup
        close_dataset(handle);
    }

    #[test]
    fn test_invalid_handle() {
        let result = write_batch(0, std::ptr::null_mut(), std::ptr::null_mut());
        assert_eq!(result, LANCE_ERROR_INVALID_HANDLE);
    }

    #[test]
    fn test_error_handling() {
        // Test null path
        let handle = create_or_open_dataset(std::ptr::null(), std::ptr::null_mut());
        assert_eq!(handle, 0);
        
        // Test invalid handle
        let result = close_dataset(0);
        assert_eq!(result, LANCE_ERROR_INVALID_HANDLE);
    }
}

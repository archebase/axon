//! Axon Lance Bridge - C FFI Interface
//!
//! This crate provides a C-compatible interface for writing data to Lance format.
//! It is a thin wrapper around the `axon-lance` core library.
//!
//! # Usage from C/C++
//!
//! ```c
//! #include <axon/lance_bridge.h>
//!
//! // Create or open a dataset
//! int64_t handle = axon_lance_create_dataset(path, schema_ptr);
//! if (handle <= 0) {
//!     const char* error = axon_lance_get_last_error();
//!     // Handle error
//! }
//!
//! // Write a batch
//! int result = axon_lance_write_batch(handle, array_ptr, schema_ptr);
//!
//! // Close the dataset
//! axon_lance_close_dataset(handle);
//! ```

use std::ffi::CStr;
use std::os::raw::{c_char, c_int};
use std::sync::Arc;
use std::convert::TryFrom;

use arrow::ffi::{FFI_ArrowArray, FFI_ArrowSchema};
use arrow_array::StructArray;

use axon_lance::{LanceWriter, LanceError};

// ============================================================================
// Error code constants for C interface
// ============================================================================

/// Operation completed successfully
pub const AXON_LANCE_SUCCESS: c_int = 0;

/// Invalid path provided
pub const AXON_LANCE_ERROR_INVALID_PATH: c_int = -1;

/// Invalid schema provided
pub const AXON_LANCE_ERROR_INVALID_SCHEMA: c_int = -2;

/// Dataset operation error
pub const AXON_LANCE_ERROR_DATASET: c_int = -3;

/// Arrow conversion error
pub const AXON_LANCE_ERROR_ARROW: c_int = -4;

/// IO error
pub const AXON_LANCE_ERROR_IO: c_int = -5;

/// Invalid dataset handle
pub const AXON_LANCE_ERROR_INVALID_HANDLE: c_int = -6;

// Legacy names for backward compatibility
pub const LANCE_SUCCESS: c_int = AXON_LANCE_SUCCESS;
pub const LANCE_ERROR_INVALID_PATH: c_int = AXON_LANCE_ERROR_INVALID_PATH;
pub const LANCE_ERROR_INVALID_SCHEMA: c_int = AXON_LANCE_ERROR_INVALID_SCHEMA;
pub const LANCE_ERROR_DATASET: c_int = AXON_LANCE_ERROR_DATASET;
pub const LANCE_ERROR_ARROW: c_int = AXON_LANCE_ERROR_ARROW;
pub const LANCE_ERROR_IO: c_int = AXON_LANCE_ERROR_IO;
pub const LANCE_ERROR_INVALID_HANDLE: c_int = AXON_LANCE_ERROR_INVALID_HANDLE;

// ============================================================================
// Error handling
// ============================================================================

use std::ffi::CString;

// Thread-local error message storage
thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<CString>> = const { std::cell::RefCell::new(None) };
}

fn set_last_error(err: &LanceError) {
    let msg = err.to_string();
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = CString::new(msg).ok();
    });
}

fn error_code(err: &LanceError) -> c_int {
    match err {
        LanceError::InvalidPath(_) => AXON_LANCE_ERROR_INVALID_PATH,
        LanceError::InvalidSchema(_) => AXON_LANCE_ERROR_INVALID_SCHEMA,
        LanceError::DatasetError(_) => AXON_LANCE_ERROR_DATASET,
        LanceError::ArrowError(_) => AXON_LANCE_ERROR_ARROW,
        LanceError::IoError(_) => AXON_LANCE_ERROR_IO,
        LanceError::InvalidHandle(_) => AXON_LANCE_ERROR_INVALID_HANDLE,
        LanceError::RuntimeError(_) => AXON_LANCE_ERROR_IO,
    }
}

/// Get the last error message as a C string
///
/// Returns a pointer to the error message string, or NULL if no error.
/// The returned string is valid until the next error occurs on this thread.
///
/// # Safety
/// The returned pointer is valid until the next call to any Axon function
/// that may set an error on this thread.
#[no_mangle]
pub extern "C" fn axon_lance_get_last_error() -> *const c_char {
    LAST_ERROR.with(|e| {
        if let Some(ref err) = *e.borrow() {
            err.as_ptr()
        } else {
            std::ptr::null()
        }
    })
}

/// Legacy name for backward compatibility
#[no_mangle]
pub extern "C" fn lance_get_last_error() -> *const c_char {
    axon_lance_get_last_error()
}

// ============================================================================
// Main FFI functions
// ============================================================================

/// Create or open a Lance dataset
///
/// Opens an existing dataset at the given path, or creates a new one if it doesn't exist.
///
/// # Arguments
/// * `path` - Path to the dataset (null-terminated C string)
/// * `schema_ptr` - Pointer to Arrow schema (FFI_ArrowSchema from Arrow C Data Interface)
///
/// # Returns
/// * Positive handle (i64) on success - use this handle for subsequent operations
/// * 0 on error - call `axon_lance_get_last_error()` for details
///
/// # Safety
/// * `path` must be a valid null-terminated C string
/// * `schema_ptr` must be a valid pointer to an FFI_ArrowSchema
#[no_mangle]
pub unsafe extern "C" fn axon_lance_create_dataset(
    path: *const c_char,
    schema_ptr: *mut FFI_ArrowSchema,
) -> i64 {
    if path.is_null() || schema_ptr.is_null() {
        set_last_error(&LanceError::InvalidPath("Null pointer provided".to_string()));
        return 0;
    }

    let path_str = match CStr::from_ptr(path).to_str() {
        Ok(s) => s,
        Err(_) => {
            set_last_error(&LanceError::InvalidPath("Invalid UTF-8 in path".to_string()));
            return 0;
        }
    };

    // Import schema from C interface
    let schema = match arrow::datatypes::Schema::try_from(&*schema_ptr) {
        Ok(s) => s,
        Err(e) => {
            set_last_error(&LanceError::ArrowError(format!("Failed to import schema: {}", e)));
            return 0;
        }
    };

    match LanceWriter::create_or_open(path_str, Arc::new(schema)) {
        Ok(handle) => handle,
        Err(e) => {
            set_last_error(&e);
            0
        }
    }
}

/// Write a RecordBatch to a Lance dataset
///
/// Appends data to the dataset. The data is passed via Apache Arrow's C Data Interface
/// for zero-copy transfer.
///
/// # Arguments
/// * `dataset_handle` - Handle returned by `axon_lance_create_dataset`
/// * `array_ptr` - Pointer to Arrow array (FFI_ArrowArray - StructArray representing RecordBatch)
/// * `schema_ptr` - Pointer to Arrow schema (FFI_ArrowSchema)
///
/// # Returns
/// * `AXON_LANCE_SUCCESS` (0) on success
/// * Negative error code on failure - call `axon_lance_get_last_error()` for details
///
/// # Safety
/// * `dataset_handle` must be a valid handle from `axon_lance_create_dataset`
/// * `array_ptr` must be a valid pointer to an FFI_ArrowArray
/// * `schema_ptr` must be a valid pointer to an FFI_ArrowSchema
#[no_mangle]
pub unsafe extern "C" fn axon_lance_write_batch(
    dataset_handle: i64,
    array_ptr: *mut FFI_ArrowArray,
    schema_ptr: *mut FFI_ArrowSchema,
) -> c_int {
    if dataset_handle <= 0 {
        set_last_error(&LanceError::InvalidHandle("Invalid dataset handle".to_string()));
        return AXON_LANCE_ERROR_INVALID_HANDLE;
    }

    if array_ptr.is_null() || schema_ptr.is_null() {
        set_last_error(&LanceError::InvalidSchema("Null pointer provided".to_string()));
        return AXON_LANCE_ERROR_INVALID_SCHEMA;
    }

    // Import Arrow data from C interface
    let record_batch = {
        // from_ffi takes ownership of the array
        let ffi_array = std::ptr::read(array_ptr);
        let ffi_schema_ref = &*schema_ptr;
        
        let array_data = match arrow::ffi::from_ffi(ffi_array, ffi_schema_ref) {
            Ok(data) => data,
            Err(e) => {
                let err = LanceError::ArrowError(format!("Failed to import Arrow data: {}", e));
                set_last_error(&err);
                return error_code(&err);
            }
        };
        
        // Convert to RecordBatch via StructArray
        let struct_array = StructArray::from(array_data);
        arrow_array::RecordBatch::from(struct_array)
    };

    match LanceWriter::write_batch(dataset_handle, record_batch) {
        Ok(_) => AXON_LANCE_SUCCESS,
        Err(e) => {
            set_last_error(&e);
            error_code(&e)
        }
    }
}

/// Close a dataset and release resources
///
/// Closes the dataset and frees all associated resources.
/// The handle becomes invalid after this call.
///
/// # Arguments
/// * `dataset_handle` - Handle returned by `axon_lance_create_dataset`
///
/// # Returns
/// * `AXON_LANCE_SUCCESS` (0) on success
/// * Negative error code on failure
#[no_mangle]
pub extern "C" fn axon_lance_close_dataset(dataset_handle: i64) -> c_int {
    if dataset_handle <= 0 {
        set_last_error(&LanceError::InvalidHandle("Invalid dataset handle".to_string()));
        return AXON_LANCE_ERROR_INVALID_HANDLE;
    }

    match LanceWriter::close(dataset_handle) {
        Ok(_) => AXON_LANCE_SUCCESS,
        Err(e) => {
            set_last_error(&e);
            error_code(&e)
        }
    }
}

// ============================================================================
// Legacy function names for backward compatibility
// ============================================================================

/// Legacy: Create or open a Lance dataset
/// Use `axon_lance_create_dataset` instead.
#[no_mangle]
pub unsafe extern "C" fn create_or_open_dataset(
    path: *const c_char,
    schema_ptr: *mut FFI_ArrowSchema,
) -> i64 {
    axon_lance_create_dataset(path, schema_ptr)
}

/// Legacy: Write a RecordBatch to a Lance dataset
/// Use `axon_lance_write_batch` instead.
#[no_mangle]
pub unsafe extern "C" fn write_batch(
    dataset_handle: i64,
    array_ptr: *mut FFI_ArrowArray,
    schema_ptr: *mut FFI_ArrowSchema,
) -> c_int {
    axon_lance_write_batch(dataset_handle, array_ptr, schema_ptr)
}

/// Legacy: Close a dataset
/// Use `axon_lance_close_dataset` instead.
#[no_mangle]
pub extern "C" fn close_dataset(dataset_handle: i64) -> c_int {
    axon_lance_close_dataset(dataset_handle)
}

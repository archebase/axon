use std::ffi::CString;
use std::os::raw::c_char;

#[derive(Debug)]
pub enum LanceError {
    InvalidPath,
    InvalidSchema,
    #[allow(dead_code)]
    DatasetError(String),
    #[allow(dead_code)]
    ArrowError(String),
    #[allow(dead_code)]
    IOError(String),
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
        LanceError::IOError(err.to_string())
    }
}

pub type Result<T> = std::result::Result<T, LanceError>;

// Error code constants for C interface
pub const LANCE_SUCCESS: i32 = 0;
pub const LANCE_ERROR_INVALID_PATH: i32 = -1;
pub const LANCE_ERROR_INVALID_SCHEMA: i32 = -2;
pub const LANCE_ERROR_DATASET: i32 = -3;
pub const LANCE_ERROR_ARROW: i32 = -4;
pub const LANCE_ERROR_IO: i32 = -5;
pub const LANCE_ERROR_INVALID_HANDLE: i32 = -6;

pub fn error_code(err: &LanceError) -> i32 {
    match err {
        LanceError::InvalidPath => LANCE_ERROR_INVALID_PATH,
        LanceError::InvalidSchema => LANCE_ERROR_INVALID_SCHEMA,
        LanceError::DatasetError(_) => LANCE_ERROR_DATASET,
        LanceError::ArrowError(_) => LANCE_ERROR_ARROW,
        LanceError::IOError(_) => LANCE_ERROR_IO,
    }
}

// Thread-local error message storage for C interface
thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<CString>> = std::cell::RefCell::new(None);
}

pub fn set_last_error(err: &LanceError) {
    let msg = format!("{:?}", err);
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = CString::new(msg).ok();
    });
}

#[no_mangle]
pub extern "C" fn lance_get_last_error() -> *const c_char {
    LAST_ERROR.with(|e| {
        if let Some(ref err) = *e.borrow() {
            err.as_ptr()
        } else {
            std::ptr::null()
        }
    })
}


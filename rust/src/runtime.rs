//! Shared Tokio runtime for Lance operations
//!
//! This module provides a global, shared Tokio runtime that is reused across
//! all Lance operations, avoiding the overhead of creating a new runtime
//! for each operation.

use once_cell::sync::Lazy;
use tokio::runtime::Runtime;

/// Global shared Tokio runtime
static RUNTIME: Lazy<Runtime> = Lazy::new(|| {
    Runtime::new().expect("Failed to create Tokio runtime")
});

/// Get a reference to the shared Tokio runtime
///
/// This function returns a reference to a multi-threaded Tokio runtime that is
/// reused for all async operations. This is much more efficient than creating
/// a new runtime for each operation.
pub fn get_runtime() -> &'static Runtime {
    &RUNTIME
}

/// Execute an async function using the shared runtime
///
/// # Example
///
/// ```ignore
/// use axon_lance::runtime::block_on;
///
/// let result = block_on(async {
///     // async code here
///     42
/// });
/// ```
pub fn block_on<F: std::future::Future>(future: F) -> F::Output {
    get_runtime().block_on(future)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_runtime_reuse() {
        let rt1 = get_runtime();
        let rt2 = get_runtime();
        // Same runtime instance
        assert!(std::ptr::eq(rt1, rt2));
    }

    #[test]
    fn test_block_on() {
        let result = block_on(async { 42 });
        assert_eq!(result, 42);
    }
}

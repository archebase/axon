//! Shared Tokio runtime for Lance operations
//!
//! This module provides a global, shared Tokio runtime that is reused across
//! all Lance operations, avoiding the overhead of creating a new runtime
//! for each operation.

use once_cell::sync::Lazy;
use tokio::runtime::Runtime;

/// Global shared Tokio runtime for general operations
static RUNTIME: Lazy<Runtime> = Lazy::new(|| {
    Runtime::new().expect("Failed to create Tokio runtime")
});

/// Dedicated multi-threaded runtime for async writes
/// Uses 8 worker threads optimized for high-throughput I/O-bound operations
/// Increased from 4 to handle concurrent writes from multiple topics
static WRITE_RUNTIME: Lazy<Runtime> = Lazy::new(|| {
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(8)
        .thread_name("lance-writer")
        .enable_all()
        .build()
        .expect("Failed to create write runtime")
});

/// Get a reference to the shared Tokio runtime
///
/// This function returns a reference to a multi-threaded Tokio runtime that is
/// reused for all async operations. This is much more efficient than creating
/// a new runtime for each operation.
pub fn get_runtime() -> &'static Runtime {
    &RUNTIME
}

/// Get a reference to the dedicated write runtime
///
/// This runtime is optimized for async write operations with multiple
/// worker threads to handle concurrent I/O.
pub fn get_write_runtime() -> &'static Runtime {
    &WRITE_RUNTIME
}

/// Spawn an async task on the write runtime (non-blocking)
///
/// Unlike `block_on`, this returns immediately and executes the
/// future in the background. Errors are logged but not propagated.
pub fn spawn_write<F>(future: F)
where
    F: std::future::Future<Output = ()> + Send + 'static,
{
    get_write_runtime().spawn(future);
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
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

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

    #[test]
    fn test_block_on_with_async_computation() {
        let result = block_on(async {
            let a = 10;
            let b = 20;
            tokio::task::yield_now().await;
            a + b
        });
        assert_eq!(result, 30);
    }

    #[test]
    fn test_runtime_from_multiple_threads() {
        use std::thread;
        
        let handles: Vec<_> = (0..4).map(|i| {
            thread::spawn(move || {
                let rt = get_runtime();
                let result = rt.block_on(async { i * 10 });
                result
            })
        }).collect();
        
        let results: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        assert_eq!(results, vec![0, 10, 20, 30]);
    }

    #[test]
    fn test_block_on_nested_spawn() {
        let result = block_on(async {
            let handle = tokio::spawn(async {
                100
            });
            handle.await.unwrap()
        });
        assert_eq!(result, 100);
    }

    #[test]
    fn test_block_on_with_sleep() {
        use std::time::{Duration, Instant};
        
        let start = Instant::now();
        block_on(async {
            tokio::time::sleep(Duration::from_millis(10)).await;
        });
        let elapsed = start.elapsed();
        
        assert!(elapsed >= Duration::from_millis(10));
    }

    #[test]
    fn test_concurrent_block_on_calls() {
        use std::thread;
        
        let counter = Arc::new(AtomicUsize::new(0));
        
        let handles: Vec<_> = (0..10).map(|_| {
            let counter_clone = counter.clone();
            thread::spawn(move || {
                block_on(async {
                    counter_clone.fetch_add(1, Ordering::SeqCst);
                });
            })
        }).collect();
        
        for h in handles {
            h.join().unwrap();
        }
        
        assert_eq!(counter.load(Ordering::SeqCst), 10);
    }

    #[test]
    fn test_block_on_returns_result() {
        let result: Result<i32, &str> = block_on(async { Ok(42) });
        assert_eq!(result.unwrap(), 42);
        
        let error: Result<i32, &str> = block_on(async { Err("error") });
        assert!(error.is_err());
    }

    #[test]
    fn test_runtime_handle() {
        let rt = get_runtime();
        let handle = rt.handle();
        
        // Can use handle to spawn tasks
        let result = handle.block_on(async { 123 });
        assert_eq!(result, 123);
    }
}

#ifndef BATCH_MANAGER_HPP
#define BATCH_MANAGER_HPP

#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/compute/api.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <functional>
#include <vector>

namespace lance_recorder {
namespace core {

/**
 * High-performance batch queue with lock-free optimizations where possible
 */
class BatchQueue {
public:
    struct BatchItem {
        std::shared_ptr<arrow::RecordBatch> batch;
        std::string dataset_path;
        int64_t dataset_handle;
        std::chrono::steady_clock::time_point enqueue_time;
        
        BatchItem() : dataset_handle(0) {}
        
        BatchItem(std::shared_ptr<arrow::RecordBatch> b, 
                 const std::string& path, 
                 int64_t handle)
            : batch(std::move(b))
            , dataset_path(path)
            , dataset_handle(handle)
            , enqueue_time(std::chrono::steady_clock::now())
        {}
    };
    
    BatchQueue() : stopped_(false), approximate_size_(0) {}
    
    ~BatchQueue() {
        stop();
    }
    
    void push(BatchItem item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(std::move(item));
            approximate_size_.store(queue_.size(), std::memory_order_relaxed);
        }
        condition_.notify_one();
    }
    
    bool pop(BatchItem& item, int timeout_ms = -1) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (timeout_ms > 0) {
            condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                [this] { return !queue_.empty() || stopped_; });
        } else {
            condition_.wait(lock, [this] { return !queue_.empty() || stopped_; });
        }
        
        if (stopped_ && queue_.empty()) {
            return false;
        }
        
        if (!queue_.empty()) {
            item = std::move(queue_.front());
            queue_.pop();
            approximate_size_.store(queue_.size(), std::memory_order_relaxed);
            return true;
        }
        
        return false;
    }
    
    size_t size() const {
        // Use atomic for fast read (approximate)
        return approximate_size_.load(std::memory_order_relaxed);
    }
    
    size_t exact_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        condition_.notify_all();
    }
    
    bool is_stopped() const {
        return stopped_.load(std::memory_order_acquire);
    }
    
private:
    std::queue<BatchItem> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::atomic<bool> stopped_;
    std::atomic<size_t> approximate_size_;  // Lock-free approximate size
};

/**
 * High-performance batch manager using Arrow Builders for zero-copy construction
 */
class BatchManager {
public:
    using WriteCallback = std::function<bool(const std::shared_ptr<arrow::RecordBatch>&, 
                                             const std::string&, int64_t)>;
    
    BatchManager(size_t batch_size, 
                 int flush_interval_ms, 
                 WriteCallback write_callback,
                 arrow::MemoryPool* memory_pool = nullptr);
    ~BatchManager();
    
    // Disable copy
    BatchManager(const BatchManager&) = delete;
    BatchManager& operator=(const BatchManager&) = delete;
    
    // Enable move
    BatchManager(BatchManager&&) = default;
    BatchManager& operator=(BatchManager&&) = default;
    
    /**
     * Initialize schema and builders
     * Must be called before add_row
     */
    void initialize_schema(std::shared_ptr<arrow::Schema> schema);
    
    /**
     * Add a row to the current batch using Arrow Builders (zero-copy)
     * @param arrays Pre-built arrays for this row
     */
    void add_row(const std::vector<std::shared_ptr<arrow::Array>>& arrays);
    
    /**
     * Add a row using raw values (builds arrays internally)
     * This is slower but more convenient
     */
    template<typename... Args>
    void add_row_values(Args&&... args);
    
    /**
     * Flush current batch immediately
     */
    void flush();
    
    /**
     * Start the background writer thread
     */
    void start();
    
    /**
     * Stop the background writer thread and flush remaining batches
     */
    void stop();
    
    /**
     * Set the dataset path and handle for writes
     */
    void set_dataset(const std::string& path, int64_t handle);
    
    /**
     * Get current batch size
     */
    size_t current_batch_size() const;
    
    /**
     * Get number of pending batches in queue
     */
    size_t pending_batches() const;
    
    /**
     * Get memory pool
     */
    arrow::MemoryPool* get_memory_pool() const;
    
private:
    void writer_thread();
    void flush_internal();
    void reset_builders();
    
    // Create builders for schema
    void create_builders();
    
    std::shared_ptr<arrow::Schema> schema_;
    std::vector<std::unique_ptr<arrow::ArrayBuilder>> builders_;
    arrow::MemoryPool* memory_pool_;
    
    size_t batch_size_;
    int flush_interval_ms_;
    WriteCallback write_callback_;
    
    BatchQueue queue_;
    std::thread writer_thread_;
    std::atomic<bool> running_;
    
    std::string dataset_path_;
    int64_t dataset_handle_;
    
    mutable std::mutex mutex_;
    std::chrono::steady_clock::time_point last_flush_time_;
    std::atomic<size_t> current_row_count_;
};

} // namespace core
} // namespace lance_recorder

#endif // BATCH_MANAGER_HPP

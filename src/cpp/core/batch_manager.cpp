#include "batch_manager.hpp"
#include <arrow/builder.h>
#include <arrow/array/builder_primitive.h>
#include <arrow/array/builder_binary.h>
#include <arrow/array/builder_nested.h>
#include <arrow/compute/api.h>
#include <arrow/array/util.h>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace axon {
namespace core {

BatchManager::BatchManager(size_t batch_size, 
                           int flush_interval_ms, 
                           WriteCallback write_callback,
                           arrow::MemoryPool* memory_pool)
    : batch_size_(batch_size)
    , flush_interval_ms_(flush_interval_ms)
    , write_callback_(write_callback)
    , memory_pool_(memory_pool ? memory_pool : arrow::default_memory_pool())
    , running_(false)
    , dataset_handle_(0)
    , last_flush_time_(std::chrono::steady_clock::now())
    , current_row_count_(0)
{
}

BatchManager::~BatchManager() {
    stop();
}

void BatchManager::set_dataset(const std::string& path, int64_t handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataset_path_ = path;
    dataset_handle_ = handle;
}

void BatchManager::initialize_schema(std::shared_ptr<arrow::Schema> schema) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (schema_) {
        throw std::runtime_error("Schema already initialized");
    }
    
    schema_ = schema;
    create_builders();
}

void BatchManager::create_builders() {
    builders_.clear();
    builders_.reserve(schema_->num_fields());
    
    for (int i = 0; i < schema_->num_fields(); ++i) {
        const auto& field = schema_->field(i);
        auto builder = arrow::MakeBuilder(field->type(), memory_pool_);
        
        if (!builder.ok()) {
            throw std::runtime_error("Failed to create builder for field " + 
                                   field->name() + ": " + builder.status().ToString());
        }
        
        builders_.push_back(std::move(builder.ValueOrDie()));
    }
}

void BatchManager::add_row(const std::vector<std::shared_ptr<arrow::Array>>& arrays) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!schema_) {
        throw std::runtime_error("Schema not initialized. Call initialize_schema() first.");
    }
    
    if (arrays.size() != static_cast<size_t>(schema_->num_fields())) {
        throw std::runtime_error("Array count mismatch. Expected " + 
                               std::to_string(schema_->num_fields()) + 
                               ", got " + std::to_string(arrays.size()));
    }
    
    // Append each array's values to corresponding builder
    // This is zero-copy when possible (Arrow handles it internally)
    for (size_t i = 0; i < arrays.size(); ++i) {
        if (!arrays[i]) {
            // Append null
            auto status = builders_[i]->AppendNull();
            if (!status.ok()) {
                std::cerr << "Warning: Failed to append null: " << status.ToString() << std::endl;
            }
            continue;
        }
        
        // Use ArrayData to create ArraySpan for AppendArraySlice
        arrow::ArraySpan span(*arrays[i]->data());
        arrow::Status status = builders_[i]->AppendArraySlice(span, 0, arrays[i]->length());
        if (!status.ok()) {
            std::cerr << "Warning: Failed to append to builder " << i 
                      << ": " << status.ToString() << std::endl;
            auto null_status = builders_[i]->AppendNull();
            if (!null_status.ok()) {
                std::cerr << "Warning: Failed to append null: " << null_status.ToString() << std::endl;
            }
        }
    }
    
    current_row_count_.store(current_row_count_.load() + 1, std::memory_order_relaxed);
    
    // Check if we need to flush
    bool should_flush = false;
    size_t current_count = current_row_count_.load(std::memory_order_relaxed);
    
    if (current_count >= batch_size_) {
        should_flush = true;
    } else if (flush_interval_ms_ > 0) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_flush_time_).count();
        if (elapsed >= flush_interval_ms_) {
            should_flush = true;
        }
    }
    
    if (should_flush) {
        flush_internal();
    }
}

void BatchManager::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    flush_internal();
}

void BatchManager::flush_internal() {
    if (!schema_ || builders_.empty()) {
        return;
    }
    
    size_t current_count = current_row_count_.load(std::memory_order_relaxed);
    if (current_count == 0) {
        return;
    }
    
    // Finish all builders and create arrays
    std::vector<std::shared_ptr<arrow::Array>> batch_arrays;
    batch_arrays.reserve(builders_.size());
    
    for (size_t i = 0; i < builders_.size(); ++i) {
        std::shared_ptr<arrow::Array> array;
        arrow::Status status = builders_[i]->Finish(&array);
        
        if (!status.ok()) {
            std::cerr << "Failed to finish builder " << i 
                      << ": " << status.ToString() << std::endl;
            // Create null array as fallback
            auto null_array = arrow::MakeArrayOfNull(schema_->field(i)->type(), current_count);
            if (null_array.ok()) {
                batch_arrays.push_back(null_array.ValueOrDie());
            } else {
                throw std::runtime_error("Failed to create null array: " + 
                                       null_array.status().ToString());
            }
        } else {
            batch_arrays.push_back(array);
        }
    }
    
    // Verify all arrays have same length
    int64_t expected_length = batch_arrays[0] ? batch_arrays[0]->length() : 0;
    for (const auto& arr : batch_arrays) {
        if (arr && arr->length() != expected_length) {
            std::cerr << "Warning: Array length mismatch in batch" << std::endl;
        }
    }
    
    // Create RecordBatch
    auto batch = arrow::RecordBatch::Make(schema_, expected_length, batch_arrays);
    
    if (!batch) {
        std::cerr << "Failed to create RecordBatch" << std::endl;
        reset_builders();
        return;
    }
    
    // Push to queue for async write
    BatchQueue::BatchItem item(batch, dataset_path_, dataset_handle_);
    queue_.push(std::move(item));
    
    // Reset for next batch
    reset_builders();
    last_flush_time_ = std::chrono::steady_clock::now();
}

void BatchManager::reset_builders() {
    // Reset all builders for next batch
    for (auto& builder : builders_) {
        builder->Reset();
    }
    current_row_count_.store(0, std::memory_order_relaxed);
}

void BatchManager::start() {
    if (running_.load(std::memory_order_acquire)) {
        return;
    }
    
    running_.store(true, std::memory_order_release);
    writer_thread_ = std::thread(&BatchManager::writer_thread, this);
}

void BatchManager::stop() {
    if (!running_.load(std::memory_order_acquire)) {
        return;
    }
    
    running_.store(false, std::memory_order_release);
    flush(); // Flush any remaining data
    
    queue_.stop();
    
    if (writer_thread_.joinable()) {
        writer_thread_.join();
    }
}

void BatchManager::writer_thread() {
    BatchQueue::BatchItem item;
    const int poll_interval_ms = 100;
    int consecutive_failures = 0;
    const int max_failures = 10;
    
    while (running_.load(std::memory_order_acquire) || !queue_.is_stopped()) {
        if (queue_.pop(item, poll_interval_ms)) {
            if (write_callback_) {
                bool success = write_callback_(item.batch, item.dataset_path, item.dataset_handle);
                if (success) {
                    consecutive_failures = 0;
                } else {
                    consecutive_failures++;
                    std::cerr << "Failed to write batch (failure " << consecutive_failures 
                              << "/" << max_failures << ")" << std::endl;
                    
                    if (consecutive_failures >= max_failures) {
                        std::cerr << "Too many consecutive failures, stopping writer thread" << std::endl;
                        break;
                    }
                }
            }
        }
    }
    
    // Drain remaining items
    while (queue_.pop(item, 0)) {
        if (write_callback_) {
            write_callback_(item.batch, item.dataset_path, item.dataset_handle);
        }
    }
}

size_t BatchManager::current_batch_size() const {
    return current_row_count_.load(std::memory_order_acquire);
}

size_t BatchManager::pending_batches() const {
    return queue_.size();
}

arrow::MemoryPool* BatchManager::get_memory_pool() const {
    return memory_pool_;
}

} // namespace core
} // namespace axon

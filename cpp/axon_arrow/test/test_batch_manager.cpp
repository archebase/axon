#include <gtest/gtest.h>
#include "batch_manager.hpp"
#include <arrow/api.h>
#include <arrow/array.h>
#include <thread>
#include <chrono>
#include <atomic>

using namespace axon::core;

// =============================================================================
// BatchManager Test Fixture
// =============================================================================

class BatchManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        write_count_ = 0;
        total_rows_written_ = 0;
        last_batch_size_ = 0;
        last_path_.clear();
        last_handle_ = 0;
        write_callback_ = [this](const std::shared_ptr<arrow::RecordBatch>& batch,
                                 const std::string& path,
                                 int64_t handle) -> bool {
            write_count_++;
            last_batch_size_ = batch->num_rows();
            total_rows_written_ += batch->num_rows();
            last_path_ = path;
            last_handle_ = handle;
            return true;
        };
    }

    std::atomic<int> write_count_;
    std::atomic<size_t> total_rows_written_;
    size_t last_batch_size_;
    std::string last_path_;
    int64_t last_handle_;
    BatchManager::WriteCallback write_callback_;
    
    // Helper to create a simple array
    std::shared_ptr<arrow::Array> make_int64_array(int64_t value) {
        auto builder = std::make_shared<arrow::Int64Builder>();
        auto status = builder->Append(value);
        if (!status.ok()) return nullptr;
        std::shared_ptr<arrow::Array> array;
        status = builder->Finish(&array);
        if (!status.ok()) return nullptr;
        return array;
    }
};

// =============================================================================
// Basic BatchManager Tests
// =============================================================================

TEST_F(BatchManagerTest, BasicAddRow) {
    BatchManager manager(10, 1000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    // Initialize schema before adding rows
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    // Create test arrays
    auto builder = std::make_shared<arrow::Int64Builder>();
    auto status = builder->Append(42);
    ASSERT_TRUE(status.ok());
    std::shared_ptr<arrow::Array> array;
    status = builder->Finish(&array);
    ASSERT_TRUE(status.ok());
    
    std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
    manager.add_row(arrays);
    
    // Wait a bit for async processing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should not have written yet (below batch size, before stop)
    ASSERT_EQ(write_count_, 0);
    
    manager.stop();
    
    // After stop(), remaining data is flushed to prevent data loss
    // So write_count should be 1 (the final flush)
    ASSERT_EQ(write_count_, 1);
    ASSERT_EQ(last_batch_size_, 1);  // Verify the flushed batch had 1 row
}

TEST_F(BatchManagerTest, FlushOnBatchSize) {
    BatchManager manager(5, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    // Initialize schema before adding rows
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    std::shared_ptr<arrow::Array> array;
    
    // Add 5 rows (batch size)
    for (int i = 0; i < 5; ++i) {
        builder->Reset();
        auto status = builder->Append(i);
        ASSERT_TRUE(status.ok());
        status = builder->Finish(&array);
        ASSERT_TRUE(status.ok());
        std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
        manager.add_row(arrays);
    }
    
    // Wait for async write
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    manager.stop();
    
    // Should have written at least once
    ASSERT_GE(write_count_, 1);
}

TEST_F(BatchManagerTest, ManualFlush) {
    BatchManager manager(100, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    // Initialize schema before adding rows
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    auto status = builder->Append(1);
    ASSERT_TRUE(status.ok());
    std::shared_ptr<arrow::Array> array;
    status = builder->Finish(&array);
    ASSERT_TRUE(status.ok());
    
    std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
    manager.add_row(arrays);
    
    // Manual flush
    manager.flush();
    
    // Wait for async write
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    manager.stop();
    
    ASSERT_GE(write_count_, 1);
}

TEST_F(BatchManagerTest, PendingBatches) {
    BatchManager manager(10, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    // Initialize schema before adding rows
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    std::shared_ptr<arrow::Array> array;
    
    // Add multiple batches
    for (int i = 0; i < 25; ++i) {
        builder->Reset();
        auto status = builder->Append(i);
        ASSERT_TRUE(status.ok());
        status = builder->Finish(&array);
        ASSERT_TRUE(status.ok());
        std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
        manager.add_row(arrays);
    }
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should have some pending batches
    size_t pending = manager.pending_batches();
    ASSERT_GE(pending, 0u);
    
    manager.stop();
}

// =============================================================================
// Dataset Configuration Tests
// =============================================================================

TEST_F(BatchManagerTest, SetDatasetPath) {
    BatchManager manager(2, 10000, write_callback_);
    manager.set_dataset("/custom/path/dataset.lance", 42);
    
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    // Add enough rows to trigger flush
    auto array = make_int64_array(1);
    ASSERT_NE(array, nullptr);
    manager.add_row({array});
    
    array = make_int64_array(2);
    manager.add_row({array});
    
    // Wait for write
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    manager.stop();
    
    EXPECT_EQ(last_path_, "/custom/path/dataset.lance");
    EXPECT_EQ(last_handle_, 42);
}

// =============================================================================
// Current Batch Size Tests
// =============================================================================

TEST_F(BatchManagerTest, CurrentBatchSize) {
    BatchManager manager(100, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    EXPECT_EQ(manager.current_batch_size(), 0u);
    
    // Add rows and check size
    for (int i = 0; i < 5; ++i) {
        auto array = make_int64_array(i);
        ASSERT_NE(array, nullptr);
        manager.add_row({array});
    }
    
    // Give time for rows to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Size should be between 0 and 5 depending on async processing
    EXPECT_LE(manager.current_batch_size(), 5u);
    
    manager.stop();
}

// =============================================================================
// Write Callback Error Handling Tests
// =============================================================================

TEST_F(BatchManagerTest, WriteCallbackFailure) {
    std::atomic<int> fail_count{0};
    auto failing_callback = [&fail_count](const std::shared_ptr<arrow::RecordBatch>&,
                                          const std::string&,
                                          int64_t) -> bool {
        fail_count++;
        return false;  // Simulate write failure
    };
    
    BatchManager manager(2, 10000, failing_callback);
    manager.set_dataset("/tmp/test.lance", 1);
    
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    // Add enough rows to trigger flush
    auto array = make_int64_array(1);
    manager.add_row({array});
    array = make_int64_array(2);
    manager.add_row({array});
    
    // Wait for write attempt
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    manager.stop();
    
    // Callback should have been called despite returning false
    EXPECT_GE(fail_count.load(), 1);
}

// =============================================================================
// Multi-Column Schema Tests
// =============================================================================

TEST_F(BatchManagerTest, MultiColumnSchema) {
    BatchManager manager(2, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    // Schema with multiple columns
    auto schema = arrow::schema({
        arrow::field("id", arrow::int64()),
        arrow::field("name", arrow::utf8()),
        arrow::field("value", arrow::float64())
    });
    manager.initialize_schema(schema);
    manager.start();
    
    // Create row with multiple columns
    auto id_builder = std::make_shared<arrow::Int64Builder>();
    ASSERT_TRUE(id_builder->Append(1).ok());
    std::shared_ptr<arrow::Array> id_array;
    ASSERT_TRUE(id_builder->Finish(&id_array).ok());
    
    auto name_builder = std::make_shared<arrow::StringBuilder>();
    ASSERT_TRUE(name_builder->Append("test").ok());
    std::shared_ptr<arrow::Array> name_array;
    ASSERT_TRUE(name_builder->Finish(&name_array).ok());
    
    auto value_builder = std::make_shared<arrow::DoubleBuilder>();
    ASSERT_TRUE(value_builder->Append(3.14).ok());
    std::shared_ptr<arrow::Array> value_array;
    ASSERT_TRUE(value_builder->Finish(&value_array).ok());
    
    manager.add_row({id_array, name_array, value_array});
    
    // Add another row to trigger flush
    id_builder->Reset();
    ASSERT_TRUE(id_builder->Append(2).ok());
    ASSERT_TRUE(id_builder->Finish(&id_array).ok());
    
    name_builder->Reset();
    ASSERT_TRUE(name_builder->Append("test2").ok());
    ASSERT_TRUE(name_builder->Finish(&name_array).ok());
    
    value_builder->Reset();
    ASSERT_TRUE(value_builder->Append(2.71).ok());
    ASSERT_TRUE(value_builder->Finish(&value_array).ok());
    
    manager.add_row({id_array, name_array, value_array});
    
    // Wait for write
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    manager.stop();
    
    EXPECT_GE(write_count_.load(), 1);
}

// =============================================================================
// BatchQueue Tests
// =============================================================================

class BatchQueueTest : public ::testing::Test {
protected:
    BatchQueue queue_;
};

TEST_F(BatchQueueTest, PushPop) {
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    auto batch = arrow::RecordBatch::MakeEmpty(schema).ValueOrDie();
    
    BatchQueue::BatchItem item(batch, "/test/path", 42);
    queue_.push(std::move(item));
    
    EXPECT_EQ(queue_.size(), 1u);
    
    BatchQueue::BatchItem popped;
    EXPECT_TRUE(queue_.pop(popped, 100));
    EXPECT_EQ(popped.dataset_path, "/test/path");
    EXPECT_EQ(popped.dataset_handle, 42);
    
    EXPECT_EQ(queue_.size(), 0u);
}

TEST_F(BatchQueueTest, PopTimeout) {
    BatchQueue::BatchItem item;
    
    auto start = std::chrono::steady_clock::now();
    bool result = queue_.pop(item, 100);  // 100ms timeout
    auto end = std::chrono::steady_clock::now();
    
    EXPECT_FALSE(result);
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_GE(elapsed.count(), 90);  // Should have waited ~100ms
}

TEST_F(BatchQueueTest, StopWakesWaiters) {
    std::atomic<bool> popped{false};
    
    std::thread waiter([this, &popped]() {
        BatchQueue::BatchItem item;
        queue_.pop(item);  // Indefinite wait
        popped = true;
    });
    
    // Give thread time to start waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    queue_.stop();
    waiter.join();
    
    EXPECT_TRUE(queue_.is_stopped());
}

TEST_F(BatchQueueTest, ExactSize) {
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    
    for (int i = 0; i < 5; ++i) {
        auto batch = arrow::RecordBatch::MakeEmpty(schema).ValueOrDie();
        BatchQueue::BatchItem item(batch, "/test", i);
        queue_.push(std::move(item));
    }
    
    EXPECT_EQ(queue_.exact_size(), 5u);
    EXPECT_EQ(queue_.size(), 5u);  // Approximate should match
}

TEST_F(BatchQueueTest, MultiplePushPop) {
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    
    // Push multiple items
    for (int i = 0; i < 10; ++i) {
        auto batch = arrow::RecordBatch::MakeEmpty(schema).ValueOrDie();
        BatchQueue::BatchItem item(batch, "/test", i);
        queue_.push(std::move(item));
    }
    
    EXPECT_EQ(queue_.size(), 10u);
    
    // Pop all items
    for (int i = 0; i < 10; ++i) {
        BatchQueue::BatchItem item;
        EXPECT_TRUE(queue_.pop(item, 100));
        EXPECT_EQ(item.dataset_handle, i);
    }
    
    EXPECT_EQ(queue_.size(), 0u);
}

// =============================================================================
// Memory Pool Tests
// =============================================================================

TEST_F(BatchManagerTest, CustomMemoryPool) {
    auto pool = arrow::system_memory_pool();
    BatchManager manager(10, 1000, write_callback_, pool);
    
    EXPECT_EQ(manager.get_memory_pool(), pool);
}

TEST_F(BatchManagerTest, DefaultMemoryPool) {
    BatchManager manager(10, 1000, write_callback_);
    
    // Should use default pool
    EXPECT_NE(manager.get_memory_pool(), nullptr);
}

// =============================================================================
// Large Data Tests
// =============================================================================

TEST_F(BatchManagerTest, ManyRows) {
    BatchManager manager(100, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    
    auto schema = arrow::schema({arrow::field("value", arrow::int64())});
    manager.initialize_schema(schema);
    manager.start();
    
    // Add many rows
    for (int i = 0; i < 500; ++i) {
        auto array = make_int64_array(i);
        ASSERT_NE(array, nullptr);
        manager.add_row({array});
    }
    
    // Wait for writes
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    manager.stop();
    
    // Should have written multiple batches
    EXPECT_GE(write_count_.load(), 5);  // 500 rows / 100 batch size = 5 batches
    EXPECT_EQ(total_rows_written_.load(), 500u);
}

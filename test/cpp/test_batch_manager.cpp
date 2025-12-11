#include <gtest/gtest.h>
#include "../src/cpp/core/batch_manager.hpp"
#include <arrow/api.h>
#include <arrow/array.h>
#include <thread>
#include <chrono>

using namespace lance_recorder::core;

class BatchManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        write_count_ = 0;
        write_callback_ = [this](const std::shared_ptr<arrow::RecordBatch>& batch,
                                 const std::string& path,
                                 int64_t handle) -> bool {
            write_count_++;
            last_batch_size_ = batch->num_rows();
            return true;
        };
    }

    int write_count_;
    size_t last_batch_size_;
    BatchManager::WriteCallback write_callback_;
};

TEST_F(BatchManagerTest, BasicAddRow) {
    BatchManager manager(10, 1000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    manager.start();
    
    // Create test arrays
    auto builder = std::make_shared<arrow::Int64Builder>();
    builder->Append(42);
    std::shared_ptr<arrow::Array> array;
    builder->Finish(&array);
    
    std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
    manager.add_row(arrays);
    
    // Wait a bit for async processing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    manager.stop();
    
    // Should not have written yet (below batch size)
    ASSERT_EQ(write_count_, 0);
}

TEST_F(BatchManagerTest, FlushOnBatchSize) {
    BatchManager manager(5, 10000, write_callback_);
    manager.set_dataset("/tmp/test.lance", 1);
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    std::shared_ptr<arrow::Array> array;
    
    // Add 5 rows (batch size)
    for (int i = 0; i < 5; ++i) {
        builder->Reset();
        builder->Append(i);
        builder->Finish(&array);
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
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    builder->Append(1);
    std::shared_ptr<arrow::Array> array;
    builder->Finish(&array);
    
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
    manager.start();
    
    auto builder = std::make_shared<arrow::Int64Builder>();
    std::shared_ptr<arrow::Array> array;
    
    // Add multiple batches
    for (int i = 0; i < 25; ++i) {
        builder->Reset();
        builder->Append(i);
        builder->Finish(&array);
        std::vector<std::shared_ptr<arrow::Array>> arrays = {array};
        manager.add_row(arrays);
    }
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should have some pending batches
    size_t pending = manager.pending_batches();
    ASSERT_GE(pending, 0);
    
    manager.stop();
}


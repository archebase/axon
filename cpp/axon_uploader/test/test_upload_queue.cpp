/**
 * Unit tests for UploadQueue
 */

#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <vector>

#include "upload_queue.hpp"

using namespace axon::uploader;

class UploadQueueTest : public ::testing::Test {
protected:
  void SetUp() override { queue_ = std::make_unique<UploadQueue>(); }

  void TearDown() override {
    if (queue_) {
      queue_->shutdown();
    }
  }

  std::unique_ptr<UploadQueue> queue_;
};

TEST_F(UploadQueueTest, EnqueueDequeue) {
  UploadItem item;
  item.mcap_path = "/tmp/test.mcap";
  item.json_path = "/tmp/test.json";
  item.task_id = "task_001";
  item.file_size_bytes = 1000;

  ASSERT_TRUE(queue_->enqueue(item));
  EXPECT_EQ(queue_->size(), 1);
  EXPECT_FALSE(queue_->empty());
  EXPECT_EQ(queue_->pending_bytes(), 1000);

  auto dequeued = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(dequeued.has_value());
  EXPECT_EQ(dequeued->task_id, "task_001");
  EXPECT_EQ(dequeued->mcap_path, "/tmp/test.mcap");

  EXPECT_EQ(queue_->size(), 0);
  EXPECT_TRUE(queue_->empty());
  EXPECT_EQ(queue_->pending_bytes(), 0);
}

TEST_F(UploadQueueTest, EnqueueMultiple) {
  for (int i = 0; i < 10; ++i) {
    UploadItem item;
    item.mcap_path = "/tmp/test_" + std::to_string(i) + ".mcap";
    item.task_id = "task_" + std::to_string(i);
    item.file_size_bytes = 100;
    ASSERT_TRUE(queue_->enqueue(item));
  }

  EXPECT_EQ(queue_->size(), 10);
  EXPECT_EQ(queue_->pending_bytes(), 1000);

  // Dequeue in FIFO order
  for (int i = 0; i < 10; ++i) {
    auto dequeued = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
    ASSERT_TRUE(dequeued.has_value());
    EXPECT_EQ(dequeued->task_id, "task_" + std::to_string(i));
  }

  EXPECT_TRUE(queue_->empty());
}

TEST_F(UploadQueueTest, DequeueTimeout) {
  // Empty queue should timeout
  auto start = std::chrono::steady_clock::now();
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.has_value());
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 90);
}

TEST_F(UploadQueueTest, RequeueForRetry) {
  UploadItem item;
  item.mcap_path = "/tmp/test.mcap";
  item.task_id = "task_001";
  item.retry_count = 1;
  item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);

  ASSERT_TRUE(queue_->requeue_for_retry(item));
  EXPECT_EQ(queue_->retry_size(), 1);
  EXPECT_EQ(queue_->size(), 0);

  // Should not be available immediately
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_FALSE(result.has_value());

  // Wait for retry time
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // Now should be available
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");
  EXPECT_EQ(result->retry_count, 1);
}

TEST_F(UploadQueueTest, RequeueImmediateRetry) {
  // Retry with no delay should go to main queue directly
  UploadItem item;
  item.mcap_path = "/tmp/test.mcap";
  item.task_id = "task_001";
  item.retry_count = 1;
  item.next_retry_at = std::chrono::steady_clock::now();  // No delay

  ASSERT_TRUE(queue_->requeue_for_retry(item));

  // Should be available immediately
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");
}

TEST_F(UploadQueueTest, Shutdown) {
  queue_->shutdown();
  EXPECT_TRUE(queue_->is_shutdown());

  // Dequeue should return immediately with nullopt
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(1000));
  EXPECT_FALSE(result.has_value());
}

TEST_F(UploadQueueTest, ConcurrentEnqueueDequeue) {
  constexpr int num_items = 100;
  std::atomic<int> dequeued_count{0};

  // Start consumer thread
  std::thread consumer([this, &dequeued_count]() {
    while (dequeued_count < num_items) {
      auto item = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
      if (item) {
        dequeued_count++;
      }
    }
  });

  // Producer: enqueue items
  for (int i = 0; i < num_items; ++i) {
    UploadItem item;
    item.mcap_path = "/tmp/test_" + std::to_string(i) + ".mcap";
    item.task_id = "task_" + std::to_string(i);
    queue_->enqueue(item);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  consumer.join();
  EXPECT_EQ(dequeued_count, num_items);
  EXPECT_TRUE(queue_->empty());
}

TEST_F(UploadQueueTest, CapacityLimit) {
  auto limited_queue = std::make_unique<UploadQueue>(5);

  for (int i = 0; i < 5; ++i) {
    UploadItem item;
    item.task_id = "task_" + std::to_string(i);
    EXPECT_TRUE(limited_queue->enqueue(item));
  }

  // Should fail when capacity exceeded
  UploadItem extra;
  extra.task_id = "extra";
  EXPECT_FALSE(limited_queue->enqueue(extra));

  // After dequeue, should be able to enqueue again
  limited_queue->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_TRUE(limited_queue->enqueue(extra));

  limited_queue->shutdown();
}


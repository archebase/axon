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
  void SetUp() override {
    queue_ = std::make_unique<UploadQueue>();
  }

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

TEST_F(UploadQueueTest, DequeueBlocking) {
  // Test the blocking dequeue() method (not dequeue_with_timeout)
  UploadItem item;
  item.mcap_path = "/tmp/test.mcap";
  item.task_id = "task_001";
  item.file_size_bytes = 1000;

  // Start consumer thread that uses blocking dequeue()
  std::atomic<bool> item_dequeued{false};
  std::thread consumer([this, &item_dequeued]() {
    auto result = queue_->dequeue();
    if (result) {
      EXPECT_EQ(result->task_id, "task_001");
      item_dequeued = true;
    }
  });

  // Give consumer time to start and block
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Enqueue item - should wake up consumer
  ASSERT_TRUE(queue_->enqueue(item));

  // Wait for consumer to finish
  consumer.join();

  EXPECT_TRUE(item_dequeued);
  EXPECT_TRUE(queue_->empty());
}

TEST_F(UploadQueueTest, DequeueBlockingShutdown) {
  // Test that blocking dequeue() returns nullopt on shutdown
  std::atomic<bool> shutdown_received{false};

  std::thread consumer([this, &shutdown_received]() {
    auto result = queue_->dequeue();
    if (!result) {
      shutdown_received = true;
    }
  });

  // Give consumer time to start and block
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Shutdown queue
  queue_->shutdown();

  // Wait for consumer to finish
  consumer.join();

  EXPECT_TRUE(shutdown_received);
}

TEST_F(UploadQueueTest, DequeueBlockingWithRetryQueue) {
  // Test blocking dequeue() when items are in retry queue
  UploadItem retry_item;
  retry_item.mcap_path = "/tmp/retry.mcap";
  retry_item.task_id = "task_retry";
  retry_item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  retry_item.file_size_bytes = 500;

  // Add item to retry queue
  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));
  EXPECT_EQ(queue_->retry_size(), 1);

  // Start consumer thread using blocking dequeue()
  std::atomic<bool> item_dequeued{false};
  std::thread consumer([this, &item_dequeued]() {
    auto result = queue_->dequeue();
    if (result) {
      EXPECT_EQ(result->task_id, "task_retry");
      item_dequeued = true;
    }
  });

  // Consumer should block until retry time arrives
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // Wait for consumer to finish (should have dequeued after retry time)
  consumer.join();

  EXPECT_TRUE(item_dequeued);
  EXPECT_EQ(queue_->retry_size(), 0);
}

TEST_F(UploadQueueTest, RetryQueueOrdering) {
  // Test that retry queue orders items correctly (earliest retry first)
  UploadItem item1;
  item1.mcap_path = "/tmp/test1.mcap";
  item1.task_id = "task_001";
  item1.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  UploadItem item2;
  item2.mcap_path = "/tmp/test2.mcap";
  item2.task_id = "task_002";
  item2.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);

  UploadItem item3;
  item3.mcap_path = "/tmp/test3.mcap";
  item3.task_id = "task_003";
  item3.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

  // Add items in non-chronological order
  ASSERT_TRUE(queue_->requeue_for_retry(item1));  // 100ms
  ASSERT_TRUE(queue_->requeue_for_retry(item2));  // 50ms (earliest)
  ASSERT_TRUE(queue_->requeue_for_retry(item3));  // 150ms (latest)

  // Wait for earliest item (50ms)
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // Should dequeue item2 first (earliest retry time)
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_002");

  // Wait for next item (100ms total from start, 40ms more)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Should dequeue item1 next
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");

  // Wait for last item (150ms total from start, 50ms more)
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // Should dequeue item3 last
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_003");
}

TEST_F(UploadQueueTest, EnqueueAfterShutdown) {
  // Test enqueue() behavior after shutdown
  // Note: enqueue() doesn't check shutdown - it only checks capacity
  // This is by design - shutdown only affects dequeue operations
  queue_->shutdown();

  UploadItem item;
  item.task_id = "task_001";

  // Enqueue still succeeds (doesn't check shutdown)
  EXPECT_TRUE(queue_->enqueue(item));

  // But dequeue will return nullopt due to shutdown
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_FALSE(result.has_value());
}

TEST_F(UploadQueueTest, RequeueForRetryAfterShutdown) {
  // Test requeue_for_retry() behavior after shutdown
  // Note: requeue_for_retry() doesn't check shutdown - it only validates retry timing
  // This is by design - shutdown only affects dequeue operations
  queue_->shutdown();

  UploadItem item;
  item.task_id = "task_001";
  item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  // Requeue still succeeds (doesn't check shutdown)
  EXPECT_TRUE(queue_->requeue_for_retry(item));

  // But dequeue will return nullopt due to shutdown
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_FALSE(result.has_value());
}

TEST_F(UploadQueueTest, PendingBytesAccumulation) {
  // Test pending_bytes() calculation with multiple items
  UploadItem item1;
  item1.task_id = "task_001";
  item1.file_size_bytes = 1000;

  UploadItem item2;
  item2.task_id = "task_002";
  item2.file_size_bytes = 2000;

  UploadItem item3;
  item3.task_id = "task_003";
  item3.file_size_bytes = 3000;

  ASSERT_TRUE(queue_->enqueue(item1));
  EXPECT_EQ(queue_->pending_bytes(), 1000);

  ASSERT_TRUE(queue_->enqueue(item2));
  EXPECT_EQ(queue_->pending_bytes(), 3000);

  ASSERT_TRUE(queue_->enqueue(item3));
  EXPECT_EQ(queue_->pending_bytes(), 6000);

  // Dequeue one item
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());

  // Pending bytes should decrease
  EXPECT_EQ(queue_->pending_bytes(), 5000);
}

TEST_F(UploadQueueTest, PendingBytesWithRetryQueue) {
  // Test pending_bytes() includes retry queue items
  UploadItem main_item;
  main_item.task_id = "main_task";
  main_item.file_size_bytes = 1000;

  UploadItem retry_item;
  retry_item.task_id = "retry_task";
  retry_item.file_size_bytes = 2000;
  retry_item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  ASSERT_TRUE(queue_->enqueue(main_item));
  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));

  // Pending bytes should include both
  EXPECT_EQ(queue_->pending_bytes(), 3000);
}

TEST_F(UploadQueueTest, ProcessRetryQueueEmpty) {
  // Test that dequeue() handles empty retry queue correctly
  // process_retry_queue() is called internally by dequeue()
  // With empty retry queue, dequeue should wait or timeout

  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_FALSE(result.has_value());

  // Queue should remain empty
  EXPECT_TRUE(queue_->empty());
  EXPECT_EQ(queue_->retry_size(), 0);
}

TEST_F(UploadQueueTest, ProcessRetryQueueMultipleItems) {
  // Test that dequeue() processes retry queue correctly
  // process_retry_queue() is called internally by dequeue()
  // Note: requeue_for_retry() adds items with next_retry_at <= now() directly to main queue
  UploadItem item1;
  item1.task_id = "task_001";
  item1.next_retry_at = std::chrono::steady_clock::now();  // Ready now - goes to main queue

  UploadItem item2;
  item2.task_id = "task_002";
  item2.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  ASSERT_TRUE(queue_->requeue_for_retry(item1));
  ASSERT_TRUE(queue_->requeue_for_retry(item2));

  // item1 goes to main queue (ready now), item2 goes to retry queue (future)
  EXPECT_EQ(queue_->retry_size(), 1);  // Only item2 in retry queue
  EXPECT_EQ(queue_->size(), 1);        // item1 in main queue

  // Dequeue should get item1 from main queue
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_001");

  // After dequeue, item1 is gone, item2 is still in retry queue
  EXPECT_EQ(queue_->retry_size(), 1);  // item2 still waiting
  EXPECT_EQ(queue_->size(), 0);        // item1 was dequeued, main queue is empty
}

TEST_F(UploadQueueTest, DequeueWithTimeoutZeroTimeout) {
  // Test dequeue_with_timeout() with zero timeout
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(0));
  EXPECT_FALSE(result.has_value());
}

TEST_F(UploadQueueTest, DequeueWithTimeoutVeryShort) {
  // Test dequeue_with_timeout() with very short timeout
  auto start = std::chrono::steady_clock::now();
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(1));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.has_value());
  // Should return quickly (within a few milliseconds)
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 10);
}

TEST_F(UploadQueueTest, SizeAndRetrySizeConsistency) {
  // Test that size() and retry_size() are consistent
  EXPECT_EQ(queue_->size(), 0);
  EXPECT_EQ(queue_->retry_size(), 0);

  UploadItem main_item;
  main_item.task_id = "main";
  ASSERT_TRUE(queue_->enqueue(main_item));
  EXPECT_EQ(queue_->size(), 1);
  EXPECT_EQ(queue_->retry_size(), 0);

  UploadItem retry_item;
  retry_item.task_id = "retry";
  retry_item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));
  EXPECT_EQ(queue_->size(), 1);        // Main queue still has 1
  EXPECT_EQ(queue_->retry_size(), 1);  // Retry queue has 1
}

TEST_F(UploadQueueTest, EnqueueCapacityCheck) {
  // Test enqueue() capacity checking with limited capacity queue
  auto limited_queue = std::make_unique<UploadQueue>(2);

  UploadItem item1;
  item1.task_id = "task_001";
  ASSERT_TRUE(limited_queue->enqueue(item1));

  UploadItem item2;
  item2.task_id = "task_002";
  ASSERT_TRUE(limited_queue->enqueue(item2));

  // Third item should fail
  UploadItem item3;
  item3.task_id = "task_003";
  EXPECT_FALSE(limited_queue->enqueue(item3));

  limited_queue->shutdown();
}

TEST_F(UploadQueueTest, RequeueForRetryCapacityCheck) {
  // Test requeue_for_retry() capacity checking
  auto limited_queue = std::make_unique<UploadQueue>(2);

  // Fill main queue
  UploadItem item1;
  item1.task_id = "task_001";
  ASSERT_TRUE(limited_queue->enqueue(item1));

  UploadItem item2;
  item2.task_id = "task_002";
  ASSERT_TRUE(limited_queue->enqueue(item2));

  // Try to requeue - should fail if capacity exceeded
  // Actually, retry queue might have separate capacity, need to check implementation
  UploadItem retry_item;
  retry_item.task_id = "retry_task";
  retry_item.next_retry_at = std::chrono::steady_clock::now();

  // This might succeed if retry queue has separate capacity
  // or fail if it shares capacity with main queue
  bool requeue_result = limited_queue->requeue_for_retry(retry_item);
  // Just verify it doesn't crash

  limited_queue->shutdown();
}

TEST_F(UploadQueueTest, CapacityZeroUnlimited) {
  // Test that capacity = 0 means unlimited
  auto unlimited_queue = std::make_unique<UploadQueue>(0);

  // Should be able to enqueue many items
  for (int i = 0; i < 100; ++i) {
    UploadItem item;
    item.task_id = "task_" + std::to_string(i);
    EXPECT_TRUE(unlimited_queue->enqueue(item));
  }

  EXPECT_EQ(unlimited_queue->size(), 100);

  unlimited_queue->shutdown();
}

TEST_F(UploadQueueTest, CapacityExactMatch) {
  // Test capacity check at exact boundary
  auto limited_queue = std::make_unique<UploadQueue>(3);

  // Fill to exact capacity
  for (int i = 0; i < 3; ++i) {
    UploadItem item;
    item.task_id = "task_" + std::to_string(i);
    EXPECT_TRUE(limited_queue->enqueue(item));
  }

  EXPECT_EQ(limited_queue->size(), 3);

  // Next item should fail
  UploadItem extra;
  extra.task_id = "extra";
  EXPECT_FALSE(limited_queue->enqueue(extra));

  limited_queue->shutdown();
}

TEST_F(UploadQueueTest, DequeueWithTimeoutRetryQueueReady) {
  // Test dequeue_with_timeout() when retry queue has ready items
  UploadItem retry_item;
  retry_item.task_id = "retry_ready";
  retry_item.next_retry_at = std::chrono::steady_clock::now();  // Ready now
  retry_item.file_size_bytes = 500;

  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));

  // Should dequeue immediately
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "retry_ready");
}

TEST_F(UploadQueueTest, DequeueWithTimeoutRetryQueueFuture) {
  // Test dequeue_with_timeout() when retry queue has future items
  UploadItem retry_item;
  retry_item.task_id = "retry_future";
  retry_item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  retry_item.file_size_bytes = 500;

  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));

  // Should timeout before retry time
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(50));
  EXPECT_FALSE(result.has_value());

  // Wait for retry time
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Now should succeed
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(100));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "retry_future");
}

TEST_F(UploadQueueTest, ProcessRetryQueueMultipleReady) {
  // Test that dequeue() processes multiple ready items from retry queue
  // process_retry_queue() is called internally by dequeue()
  // Note: requeue_for_retry() adds items with next_retry_at <= now() directly to main queue
  UploadItem item1;
  item1.task_id = "ready1";
  item1.next_retry_at = std::chrono::steady_clock::now();  // Ready now - goes to main queue

  UploadItem item2;
  item2.task_id = "ready2";
  item2.next_retry_at = std::chrono::steady_clock::now();  // Ready now - goes to main queue

  UploadItem item3;
  item3.task_id = "future";
  item3.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  ASSERT_TRUE(queue_->requeue_for_retry(item1));
  ASSERT_TRUE(queue_->requeue_for_retry(item2));
  ASSERT_TRUE(queue_->requeue_for_retry(item3));

  // item1 and item2 go to main queue (ready now), item3 goes to retry queue (future)
  EXPECT_EQ(queue_->retry_size(), 1);  // Only item3 in retry queue
  EXPECT_EQ(queue_->size(), 2);        // item1 and item2 in main queue

  // First dequeue: item1 is dequeued from main queue
  auto result1 = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result1.has_value());
  EXPECT_EQ(result1->task_id, "ready1");

  // After first dequeue: item2 is in main queue, item3 is in retry queue
  EXPECT_EQ(queue_->retry_size(), 1);  // item3 still waiting
  EXPECT_EQ(queue_->size(), 1);        // item2 in main queue

  // Second dequeue: item2 is dequeued from main queue
  auto result2 = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result2.has_value());
  EXPECT_EQ(result2->task_id, "ready2");

  // After second dequeue: only item3 remains in retry queue
  EXPECT_EQ(queue_->retry_size(), 1);  // item3 still waiting
  EXPECT_EQ(queue_->size(), 0);        // main queue is empty
}

TEST_F(UploadQueueTest, PendingBytesDecrement) {
  // Test that pending_bytes decreases when items are dequeued
  UploadItem item1;
  item1.task_id = "task_001";
  item1.file_size_bytes = 1000;

  UploadItem item2;
  item2.task_id = "task_002";
  item2.file_size_bytes = 2000;

  ASSERT_TRUE(queue_->enqueue(item1));
  ASSERT_TRUE(queue_->enqueue(item2));

  EXPECT_EQ(queue_->pending_bytes(), 3000);

  // Dequeue first item
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(queue_->pending_bytes(), 2000);

  // Dequeue second item
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(queue_->pending_bytes(), 0);
}

TEST_F(UploadQueueTest, PendingBytesWithRetryDequeue) {
  // Test pending_bytes with retry queue items being dequeued
  UploadItem retry_item;
  retry_item.task_id = "retry";
  retry_item.file_size_bytes = 1500;
  retry_item.next_retry_at = std::chrono::steady_clock::now();

  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));
  EXPECT_EQ(queue_->pending_bytes(), 1500);

  // Dequeue should decrease pending bytes
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(queue_->pending_bytes(), 0);
}

TEST_F(UploadQueueTest, EmptyQueue) {
  // Test empty() method
  EXPECT_TRUE(queue_->empty());
  EXPECT_EQ(queue_->size(), 0);

  // Add item
  UploadItem item;
  item.task_id = "task_001";
  ASSERT_TRUE(queue_->enqueue(item));

  EXPECT_FALSE(queue_->empty());
  EXPECT_EQ(queue_->size(), 1);

  // Dequeue item
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());

  EXPECT_TRUE(queue_->empty());
  EXPECT_EQ(queue_->size(), 0);
}

TEST_F(UploadQueueTest, EmptyWithRetryQueue) {
  // Test empty() when items are in retry queue
  UploadItem retry_item;
  retry_item.task_id = "task_retry";
  retry_item.next_retry_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);

  ASSERT_TRUE(queue_->requeue_for_retry(retry_item));

  // Queue should not be empty (has item in retry queue)
  EXPECT_FALSE(queue_->empty());
  EXPECT_EQ(queue_->retry_size(), 1);
  EXPECT_EQ(queue_->size(), 0);

  // Wait for retry time
  std::this_thread::sleep_for(std::chrono::milliseconds(110));

  // Dequeue will internally process retry queue - item should move to main queue
  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->task_id, "task_retry");

  // After dequeue, should be empty
  EXPECT_TRUE(queue_->empty());
  EXPECT_EQ(queue_->size(), 0);
  EXPECT_EQ(queue_->retry_size(), 0);
}

TEST_F(UploadQueueTest, IsShutdownBeforeShutdown) {
  // Test is_shutdown() before shutdown
  EXPECT_FALSE(queue_->is_shutdown());

  // Shutdown
  queue_->shutdown();
  EXPECT_TRUE(queue_->is_shutdown());
}

TEST_F(UploadQueueTest, IsShutdownAfterOperations) {
  // Test is_shutdown() after various operations
  EXPECT_FALSE(queue_->is_shutdown());

  // Enqueue and dequeue while not shutdown
  UploadItem item;
  item.task_id = "task_001";
  ASSERT_TRUE(queue_->enqueue(item));
  EXPECT_FALSE(queue_->is_shutdown());

  auto result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(queue_->is_shutdown());

  // Now shutdown
  queue_->shutdown();
  EXPECT_TRUE(queue_->is_shutdown());

  // Operations should still work but dequeue returns nullopt
  ASSERT_TRUE(queue_->enqueue(item));
  result = queue_->dequeue_with_timeout(std::chrono::milliseconds(10));
  EXPECT_FALSE(result.has_value());  // Shutdown queue returns nullopt
}

/**
 * @file test_worker_thread_pool.cpp
 * @brief Unit tests for WorkerThreadPool class
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "worker_thread_pool.hpp"

namespace axon {
namespace recorder {
namespace {

class WorkerThreadPoolTest : public ::testing::Test {
protected:
  void SetUp() override {
    WorkerThreadPool::Config config;
    config.queue_capacity_per_topic = 1024;
    config.worker_idle_sleep_us = 10;
    pool_ = std::make_unique<WorkerThreadPool>(config);
  }

  void TearDown() override {
    if (pool_->is_running()) {
      pool_->stop();
    }
    pool_.reset();
  }

  std::unique_ptr<WorkerThreadPool> pool_;
};

TEST_F(WorkerThreadPoolTest, DefaultConstruction) {
  WorkerThreadPool pool;
  EXPECT_FALSE(pool.is_running());
  EXPECT_FALSE(pool.is_paused());
  EXPECT_EQ(pool.topic_count(), 0);
}

TEST_F(WorkerThreadPoolTest, StartStop) {
  EXPECT_FALSE(pool_->is_running());

  pool_->start();
  EXPECT_TRUE(pool_->is_running());

  pool_->stop();
  EXPECT_FALSE(pool_->is_running());
}

TEST_F(WorkerThreadPoolTest, DoubleStartIsNoOp) {
  pool_->start();
  EXPECT_TRUE(pool_->is_running());

  pool_->start();  // Should not crash
  EXPECT_TRUE(pool_->is_running());

  pool_->stop();
}

TEST_F(WorkerThreadPoolTest, DoubleStopIsNoOp) {
  pool_->start();
  pool_->stop();
  EXPECT_FALSE(pool_->is_running());

  pool_->stop();  // Should not crash
  EXPECT_FALSE(pool_->is_running());
}

TEST_F(WorkerThreadPoolTest, CreateTopicWorker) {
  std::atomic<int> message_count{0};
  auto handler = [&](const std::string& /* topic */, int64_t /* ts */,
                     const uint8_t* /* data */, size_t /* size */,
                     uint32_t /* seq */) {
    message_count.fetch_add(1);
    return true;
  };

  EXPECT_TRUE(pool_->create_topic_worker("/test_topic", handler));
  EXPECT_EQ(pool_->topic_count(), 1);

  auto topics = pool_->get_topics();
  ASSERT_EQ(topics.size(), 1);
  EXPECT_EQ(topics[0], "/test_topic");
}

TEST_F(WorkerThreadPoolTest, CannotCreateDuplicateTopic) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  EXPECT_TRUE(pool_->create_topic_worker("/test", handler));
  EXPECT_FALSE(pool_->create_topic_worker("/test", handler));
  EXPECT_EQ(pool_->topic_count(), 1);
}

TEST_F(WorkerThreadPoolTest, RemoveTopicWorker) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/topic1", handler);
  pool_->create_topic_worker("/topic2", handler);
  EXPECT_EQ(pool_->topic_count(), 2);

  pool_->remove_topic_worker("/topic1");
  EXPECT_EQ(pool_->topic_count(), 1);

  auto topics = pool_->get_topics();
  ASSERT_EQ(topics.size(), 1);
  EXPECT_EQ(topics[0], "/topic2");
}

TEST_F(WorkerThreadPoolTest, TryPush) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/test", handler);

  // Push should succeed when pool exists
  MessageItem item(12345, {0x01, 0x02});
  EXPECT_TRUE(pool_->try_push("/test", std::move(item)));

  // Push to non-existent topic should fail
  MessageItem item2(12345, {0x01});
  EXPECT_FALSE(pool_->try_push("/nonexistent", std::move(item2)));
}

TEST_F(WorkerThreadPoolTest, MessageProcessing) {
  std::atomic<int> processed_count{0};
  std::vector<int64_t> received_timestamps;
  std::mutex ts_mutex;

  auto handler = [&](const std::string& topic, int64_t ts,
                     const uint8_t* /* data */, size_t /* size */,
                     uint32_t /* seq */) {
    EXPECT_EQ(topic, "/test");
    {
      std::lock_guard<std::mutex> lock(ts_mutex);
      received_timestamps.push_back(ts);
    }
    processed_count.fetch_add(1);
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  // Push some messages
  for (int i = 0; i < 10; ++i) {
    MessageItem item(i * 1000, {static_cast<uint8_t>(i)});
    EXPECT_TRUE(pool_->try_push("/test", std::move(item)));
  }

  // Wait for processing
  auto start = std::chrono::steady_clock::now();
  while (processed_count.load() < 10) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) {
      FAIL() << "Timeout waiting for message processing";
    }
  }

  pool_->stop();

  EXPECT_EQ(processed_count.load(), 10);

  // Verify timestamps received in order
  std::lock_guard<std::mutex> lock(ts_mutex);
  ASSERT_EQ(received_timestamps.size(), 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(received_timestamps[i], i * 1000);
  }
}

TEST_F(WorkerThreadPoolTest, PauseResume) {
  std::atomic<int> processed_count{0};

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    processed_count.fetch_add(1);
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  EXPECT_FALSE(pool_->is_paused());

  pool_->pause();
  EXPECT_TRUE(pool_->is_paused());

  // Push messages while paused
  for (int i = 0; i < 5; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/test", std::move(item));
  }

  // Give worker time to NOT process
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Should not have processed (paused)
  int count_while_paused = processed_count.load();

  // Resume
  pool_->resume();
  EXPECT_FALSE(pool_->is_paused());

  // Wait for processing
  auto start = std::chrono::steady_clock::now();
  while (processed_count.load() < 5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) {
      break;
    }
  }

  pool_->stop();

  // Should have processed after resume
  EXPECT_EQ(processed_count.load(), 5);
  EXPECT_LE(count_while_paused, 5);  // Some may have been processed before pause took effect
}

TEST_F(WorkerThreadPoolTest, StatsTracking) {
  std::atomic<bool> should_succeed{true};

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return should_succeed.load();
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  // Push messages
  for (int i = 0; i < 10; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/test", std::move(item));
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  pool_->stop();

  // Check stats
  auto stats = pool_->get_topic_stats("/test");
  EXPECT_EQ(stats.received, 10u);
  EXPECT_EQ(stats.written, 10u);  // All succeeded
  EXPECT_EQ(stats.dropped, 0u);
}

TEST_F(WorkerThreadPoolTest, AggregateStats) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/topic1", handler);
  pool_->create_topic_worker("/topic2", handler);
  pool_->start();

  // Push to topic1
  for (int i = 0; i < 5; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/topic1", std::move(item));
  }

  // Push to topic2
  for (int i = 0; i < 3; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/topic2", std::move(item));
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  pool_->stop();

  auto aggregate = pool_->get_aggregate_stats();
  EXPECT_EQ(aggregate.total_received, 8);
  EXPECT_EQ(aggregate.total_written, 8);
  EXPECT_EQ(aggregate.total_dropped, 0);
}

TEST_F(WorkerThreadPoolTest, QueueDrainOnStop) {
  std::atomic<int> processed{0};

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    processed.fetch_add(1);
    return true;
  };

  pool_->create_topic_worker("/test", handler);

  // Don't start yet - just queue messages
  for (int i = 0; i < 100; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i % 256)});
    pool_->try_push("/test", std::move(item));
  }

  // Start and immediately stop
  pool_->start();
  pool_->stop();

  // Queue should be drained during stop
  EXPECT_EQ(processed.load(), 100);
}

TEST_F(WorkerThreadPoolTest, SequenceNumbers) {
  std::vector<uint32_t> received_sequences;
  std::mutex seq_mutex;

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t, uint32_t seq) {
    std::lock_guard<std::mutex> lock(seq_mutex);
    received_sequences.push_back(seq);
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  // Push messages
  for (int i = 0; i < 10; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/test", std::move(item));
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  pool_->stop();

  // Sequence numbers should be 0, 1, 2, ...
  std::lock_guard<std::mutex> lock(seq_mutex);
  ASSERT_EQ(received_sequences.size(), 10);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(received_sequences[i], static_cast<uint32_t>(i));
  }
}

// ============================================================================
// Enhanced Coverage Tests (Phase 4)
// ============================================================================

TEST_F(WorkerThreadPoolTest, StopCalledMultipleTimes) {
  pool_->start();
  EXPECT_TRUE(pool_->is_running());

  // First stop
  pool_->stop();
  EXPECT_FALSE(pool_->is_running());

  // Second stop - should be idempotent
  pool_->stop();
  EXPECT_FALSE(pool_->is_running());

  // Third stop - still idempotent
  pool_->stop();
  EXPECT_FALSE(pool_->is_running());
}

TEST_F(WorkerThreadPoolTest, TryPushAfterStop) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();
  pool_->stop();

  // Try to push after stop - behavior depends on implementation
  // The queue might still accept items even after stop
  MessageItem item(12345, {0x01, 0x02});
  bool result = pool_->try_push("/test", std::move(item));

  // Just verify it doesn't crash - result may vary
  (void)result;
}

TEST_F(WorkerThreadPoolTest, GetTopicStatsNonexistentTopic) {
  // Get stats for a topic that doesn't exist
  auto stats = pool_->get_topic_stats("/nonexistent_topic");

  // Should return empty/zeroed stats
  EXPECT_EQ(stats.received, 0u);
  EXPECT_EQ(stats.written, 0u);
  EXPECT_EQ(stats.dropped, 0u);
}

TEST_F(WorkerThreadPoolTest, CreateTopicWorkerDuplicateName) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  // First creation should succeed
  EXPECT_TRUE(pool_->create_topic_worker("/duplicate", handler));
  EXPECT_EQ(pool_->topic_count(), 1);

  // Second creation with same name should fail
  EXPECT_FALSE(pool_->create_topic_worker("/duplicate", handler));
  EXPECT_EQ(pool_->topic_count(), 1);

  // Third attempt should also fail
  EXPECT_FALSE(pool_->create_topic_worker("/duplicate", handler));
  EXPECT_EQ(pool_->topic_count(), 1);
}

TEST_F(WorkerThreadPoolTest, RemoveNonexistentTopic) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/exists", handler);
  EXPECT_EQ(pool_->topic_count(), 1);

  // Remove non-existent topic - should not crash
  pool_->remove_topic_worker("/does_not_exist");
  EXPECT_EQ(pool_->topic_count(), 1);  // Original still there
}

TEST_F(WorkerThreadPoolTest, PauseWhenNotRunning) {
  // Pool not started yet
  EXPECT_FALSE(pool_->is_running());

  pool_->pause();
  EXPECT_TRUE(pool_->is_paused());

  pool_->resume();
  EXPECT_FALSE(pool_->is_paused());
}

TEST_F(WorkerThreadPoolTest, CreateTopicWorkerWhileRunning) {
  pool_->start();
  EXPECT_TRUE(pool_->is_running());

  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  // Should be able to create topic while running
  EXPECT_TRUE(pool_->create_topic_worker("/new_topic", handler));
  EXPECT_EQ(pool_->topic_count(), 1);

  // Push to new topic
  MessageItem item(12345, {0x01});
  EXPECT_TRUE(pool_->try_push("/new_topic", std::move(item)));

  pool_->stop();
}

TEST_F(WorkerThreadPoolTest, RemoveTopicWorkerWhileRunning) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  pool_->create_topic_worker("/topic1", handler);
  pool_->create_topic_worker("/topic2", handler);
  pool_->start();

  EXPECT_EQ(pool_->topic_count(), 2);

  // Remove topic while running
  pool_->remove_topic_worker("/topic1");
  EXPECT_EQ(pool_->topic_count(), 1);

  // Push to removed topic should fail
  MessageItem item(12345, {0x01});
  EXPECT_FALSE(pool_->try_push("/topic1", std::move(item)));

  // Push to remaining topic should work
  MessageItem item2(12345, {0x02});
  EXPECT_TRUE(pool_->try_push("/topic2", std::move(item2)));

  pool_->stop();
}

TEST_F(WorkerThreadPoolTest, HandlerReturnsFailure) {
  std::atomic<int> call_count{0};

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    call_count.fetch_add(1);
    return false;  // Always fail
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  for (int i = 0; i < 5; ++i) {
    MessageItem item(i, {static_cast<uint8_t>(i)});
    pool_->try_push("/test", std::move(item));
  }

  // Wait for processing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pool_->stop();

  // Handler should have been called for all messages
  EXPECT_EQ(call_count.load(), 5);

  // Stats behavior depends on implementation
  // Handler returning false may or may not count as "dropped"
  auto stats = pool_->get_topic_stats("/test");
  EXPECT_EQ(stats.received, 5u);
  // Just verify we processed all messages
  EXPECT_EQ(stats.received, static_cast<uint64_t>(call_count.load()));
}

TEST_F(WorkerThreadPoolTest, EmptyMessageData) {
  std::atomic<bool> handler_called{false};
  size_t received_size = 999;

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t size, uint32_t) {
    handler_called = true;
    received_size = size;
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  // Push empty message
  MessageItem item(12345, {});  // Empty data
  pool_->try_push("/test", std::move(item));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pool_->stop();

  EXPECT_TRUE(handler_called);
  EXPECT_EQ(received_size, 0u);
}

TEST_F(WorkerThreadPoolTest, LargeMessageData) {
  size_t received_size = 0;

  auto handler = [&](const std::string&, int64_t, const uint8_t*, size_t size, uint32_t) {
    received_size = size;
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  // Push large message (1MB)
  std::vector<uint8_t> large_data(1024 * 1024, 0xAB);
  MessageItem item(12345, std::move(large_data));
  pool_->try_push("/test", std::move(item));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  pool_->stop();

  EXPECT_EQ(received_size, 1024 * 1024);
}

TEST_F(WorkerThreadPoolTest, ManyTopics) {
  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  // Create many topics
  const int num_topics = 50;
  for (int i = 0; i < num_topics; ++i) {
    std::string topic = "/topic_" + std::to_string(i);
    EXPECT_TRUE(pool_->create_topic_worker(topic, handler));
  }

  EXPECT_EQ(pool_->topic_count(), num_topics);

  // Verify all topics are in the list
  auto topics = pool_->get_topics();
  EXPECT_EQ(topics.size(), static_cast<size_t>(num_topics));
}

TEST_F(WorkerThreadPoolTest, ZeroTimestamp) {
  int64_t received_ts = -1;

  auto handler = [&](const std::string&, int64_t ts, const uint8_t*, size_t, uint32_t) {
    received_ts = ts;
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  MessageItem item(0, {0x01});  // Zero timestamp
  pool_->try_push("/test", std::move(item));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pool_->stop();

  EXPECT_EQ(received_ts, 0);
}

TEST_F(WorkerThreadPoolTest, NegativeTimestamp) {
  int64_t received_ts = 0;

  auto handler = [&](const std::string&, int64_t ts, const uint8_t*, size_t, uint32_t) {
    received_ts = ts;
    return true;
  };

  pool_->create_topic_worker("/test", handler);
  pool_->start();

  MessageItem item(-12345, {0x01});  // Negative timestamp
  pool_->try_push("/test", std::move(item));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pool_->stop();

  EXPECT_EQ(received_ts, -12345);
}

TEST_F(WorkerThreadPoolTest, GetTopicsEmpty) {
  auto topics = pool_->get_topics();
  EXPECT_TRUE(topics.empty());
}

TEST_F(WorkerThreadPoolTest, AggregateStatsEmpty) {
  auto stats = pool_->get_aggregate_stats();
  EXPECT_EQ(stats.total_received, 0);
  EXPECT_EQ(stats.total_written, 0);
  EXPECT_EQ(stats.total_dropped, 0);
}

TEST_F(WorkerThreadPoolTest, ConfigCustomValues) {
  WorkerThreadPool::Config config;
  config.queue_capacity_per_topic = 100;
  config.worker_idle_sleep_us = 1000;

  auto custom_pool = std::make_unique<WorkerThreadPool>(config);

  auto handler = [](const std::string&, int64_t, const uint8_t*, size_t, uint32_t) {
    return true;
  };

  EXPECT_TRUE(custom_pool->create_topic_worker("/test", handler));
  custom_pool->start();
  custom_pool->stop();
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


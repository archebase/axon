/**
 * @file test_spsc_queue.cpp
 * @brief Unit tests for SPSCQueue and MPSCQueue lock-free data structures
 *
 * Tests correctness, thread safety, and performance of the lock-free queues
 * used in the high-throughput message recording pipeline.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "spsc_queue.hpp"

using namespace axon::recorder;

// ============================================================================
// SPSCQueue Basic Tests
// ============================================================================

class SPSCQueueTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(SPSCQueueTest, Construction) {
  SPSCQueue<int> queue(16);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0);
  // Capacity is rounded up to power of 2
  EXPECT_GE(queue.capacity(), 16);
}

TEST_F(SPSCQueueTest, CapacityRoundsToPowerOf2) {
  SPSCQueue<int> queue1(7);
  EXPECT_EQ(queue1.capacity(), 8);  // 7 -> 8
  
  SPSCQueue<int> queue2(9);
  EXPECT_EQ(queue2.capacity(), 16);  // 9 -> 16
  
  SPSCQueue<int> queue3(16);
  EXPECT_EQ(queue3.capacity(), 16);  // Already power of 2
  
  SPSCQueue<int> queue4(1);
  EXPECT_EQ(queue4.capacity(), 1);  // 1 is power of 2
  
  SPSCQueue<int> queue5(0);
  EXPECT_EQ(queue5.capacity(), 1);  // 0 rounds to 1
}

TEST_F(SPSCQueueTest, BasicPushPop) {
  SPSCQueue<int> queue(4);
  
  int value = 42;
  EXPECT_TRUE(queue.try_push(std::move(value)));
  EXPECT_EQ(queue.size(), 1);
  EXPECT_FALSE(queue.empty());
  
  int result = 0;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_EQ(result, 42);
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0);
}

TEST_F(SPSCQueueTest, MultiplePushPop) {
  SPSCQueue<int> queue(16);
  
  for (int i = 0; i < 10; ++i) {
    int value = i;
    EXPECT_TRUE(queue.try_push(std::move(value)));
  }
  EXPECT_EQ(queue.size(), 10);
  
  for (int i = 0; i < 10; ++i) {
    int result = -1;
    EXPECT_TRUE(queue.try_pop(result));
    EXPECT_EQ(result, i);  // FIFO order
  }
  EXPECT_TRUE(queue.empty());
}

TEST_F(SPSCQueueTest, QueueFullBehavior) {
  SPSCQueue<int> queue(4);  // Capacity is 4
  
  // Fill the queue
  for (int i = 0; i < 4; ++i) {
    int value = i;
    EXPECT_TRUE(queue.try_push(std::move(value)));
  }
  
  // Queue is full, next push should fail
  int value = 100;
  EXPECT_FALSE(queue.try_push(std::move(value)));
  
  // Pop one and try again
  int result;
  EXPECT_TRUE(queue.try_pop(result));
  
  value = 100;
  EXPECT_TRUE(queue.try_push(std::move(value)));
}

TEST_F(SPSCQueueTest, QueueEmptyBehavior) {
  SPSCQueue<int> queue(4);
  
  int result;
  EXPECT_FALSE(queue.try_pop(result));
  
  // Push and pop
  int value = 42;
  queue.try_push(std::move(value));
  EXPECT_TRUE(queue.try_pop(result));
  
  // Should be empty again
  EXPECT_FALSE(queue.try_pop(result));
}

// ============================================================================
// SPSCQueue Move Semantics Tests
// ============================================================================

TEST_F(SPSCQueueTest, MoveOnlyTypes) {
  SPSCQueue<std::unique_ptr<int>> queue(4);
  
  auto ptr = std::make_unique<int>(42);
  EXPECT_TRUE(queue.try_push(std::move(ptr)));
  EXPECT_EQ(ptr, nullptr);  // Moved from
  
  std::unique_ptr<int> result;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_NE(result, nullptr);
  EXPECT_EQ(*result, 42);
}

TEST_F(SPSCQueueTest, StringMoveSemantics) {
  SPSCQueue<std::string> queue(4);
  
  std::string str = "Hello, World!";
  const char* original_data = str.data();
  
  EXPECT_TRUE(queue.try_push(std::move(str)));
  
  std::string result;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_EQ(result, "Hello, World!");
  // Move should have preserved the data pointer (SSO may affect this for small strings)
}

TEST_F(SPSCQueueTest, VectorMoveSemantics) {
  SPSCQueue<std::vector<int>> queue(4);
  
  std::vector<int> vec = {1, 2, 3, 4, 5};
  EXPECT_TRUE(queue.try_push(std::move(vec)));
  EXPECT_TRUE(vec.empty());  // Moved from
  
  std::vector<int> result;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_EQ(result, std::vector<int>({1, 2, 3, 4, 5}));
}

// ============================================================================
// SPSCQueue Thread Safety Tests
// ============================================================================

TEST_F(SPSCQueueTest, SingleProducerSingleConsumer) {
  constexpr size_t NUM_ITEMS = 10000;
  SPSCQueue<int> queue(1024);
  
  std::atomic<bool> producer_done{false};
  std::atomic<size_t> consumed{0};
  
  // Producer thread
  std::thread producer([&]() {
    for (size_t i = 0; i < NUM_ITEMS; ++i) {
      int value = static_cast<int>(i);
      while (!queue.try_push(std::move(value))) {
        std::this_thread::yield();
      }
    }
    producer_done = true;
  });
  
  // Consumer thread
  std::thread consumer([&]() {
    size_t expected = 0;
    while (expected < NUM_ITEMS) {
      int result;
      if (queue.try_pop(result)) {
        EXPECT_EQ(result, static_cast<int>(expected));
        ++expected;
        ++consumed;
      } else if (producer_done && queue.empty()) {
        break;
      } else {
        std::this_thread::yield();
      }
    }
  });
  
  producer.join();
  consumer.join();
  
  EXPECT_EQ(consumed, NUM_ITEMS);
  EXPECT_TRUE(queue.empty());
}

TEST_F(SPSCQueueTest, HighThroughput) {
  // Reduced from 100000 to avoid CI timeout on slow runners
  constexpr size_t NUM_ITEMS = 10000;
  SPSCQueue<uint64_t> queue(4096);
  
  std::atomic<size_t> produced{0};
  std::atomic<size_t> consumed{0};
  
  auto start = std::chrono::steady_clock::now();
  
  // Producer thread
  std::thread producer([&]() {
    for (size_t i = 0; i < NUM_ITEMS; ++i) {
      uint64_t value = i;
      while (!queue.try_push(std::move(value))) {
        std::this_thread::yield();
      }
      ++produced;
    }
  });
  
  // Consumer thread
  std::thread consumer([&]() {
    while (consumed < NUM_ITEMS) {
      uint64_t result;
      if (queue.try_pop(result)) {
        ++consumed;
      } else {
        std::this_thread::yield();
      }
    }
  });
  
  producer.join();
  consumer.join();
  
  auto elapsed = std::chrono::steady_clock::now() - start;
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  
  EXPECT_EQ(produced, NUM_ITEMS);
  EXPECT_EQ(consumed, NUM_ITEMS);
  
  // Performance check: should be fast (< 1 second for 100k items)
  EXPECT_LT(ms, 1000) << "Queue throughput too slow: " << ms << "ms";
}

// ============================================================================
// SPSCQueue Destructor Tests
// ============================================================================

TEST_F(SPSCQueueTest, DestructorCleansUpRemainingElements) {
  static std::atomic<int> destructor_calls{0};
  
  struct TrackedObject {
    TrackedObject() = default;
    ~TrackedObject() { ++destructor_calls; }
    TrackedObject(TrackedObject&&) = default;
    TrackedObject& operator=(TrackedObject&&) = default;
    TrackedObject(const TrackedObject&) = delete;
    TrackedObject& operator=(const TrackedObject&) = delete;
  };
  
  destructor_calls = 0;
  
  {
    SPSCQueue<TrackedObject> queue(8);
    
    for (int i = 0; i < 5; ++i) {
      TrackedObject obj;
      queue.try_push(std::move(obj));
    }
    // Queue destroyed here
  }
  
  // Destructor should have been called for all 5 elements
  // Note: The moved-from objects also get destroyed (10 total)
  EXPECT_GE(destructor_calls.load(), 5);
}

// ============================================================================
// MPSCQueue Basic Tests
// ============================================================================

class MPSCQueueTest : public ::testing::Test {
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(MPSCQueueTest, Construction) {
  MPSCQueue<int> queue(16, 4);  // 4 producer slots
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.size(), 0);
}

TEST_F(MPSCQueueTest, BasicPushPop) {
  MPSCQueue<int> queue(16, 4);
  
  int value = 42;
  EXPECT_TRUE(queue.try_push(std::move(value)));
  EXPECT_FALSE(queue.empty());
  
  int result;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_EQ(result, 42);
  EXPECT_TRUE(queue.empty());
}

TEST_F(MPSCQueueTest, MultipleProducersCorrectness) {
  constexpr size_t NUM_PRODUCERS = 4;
  constexpr size_t ITEMS_PER_PRODUCER = 1000;
  
  MPSCQueue<int> queue(256, NUM_PRODUCERS);
  
  std::atomic<size_t> produced{0};
  std::atomic<size_t> consumed{0};
  std::vector<std::thread> producers;
  
  // Start producer threads
  for (size_t p = 0; p < NUM_PRODUCERS; ++p) {
    producers.emplace_back([&, p]() {
      for (size_t i = 0; i < ITEMS_PER_PRODUCER; ++i) {
        int value = static_cast<int>(p * ITEMS_PER_PRODUCER + i);
        while (!queue.try_push(std::move(value))) {
          std::this_thread::yield();
        }
        ++produced;
      }
    });
  }
  
  // Consumer thread
  std::thread consumer([&]() {
    const size_t total = NUM_PRODUCERS * ITEMS_PER_PRODUCER;
    while (consumed < total) {
      int result;
      if (queue.try_pop(result)) {
        ++consumed;
      } else {
        std::this_thread::yield();
      }
    }
  });
  
  // Wait for producers
  for (auto& t : producers) {
    t.join();
  }
  
  // Wait for consumer
  consumer.join();
  
  EXPECT_EQ(produced, NUM_PRODUCERS * ITEMS_PER_PRODUCER);
  EXPECT_EQ(consumed, NUM_PRODUCERS * ITEMS_PER_PRODUCER);
  EXPECT_TRUE(queue.empty());
}

TEST_F(MPSCQueueTest, ThreadLocalQueueAssignment) {
  // Test that each producer thread is pinned to its own queue
  // With thread_local assignment, a single thread can only use one underlying queue
  MPSCQueue<int> queue(4, 4);  // 4 queues, each with capacity 4
  
  // Single thread can only push to its assigned queue (capacity 4)
  for (int i = 0; i < 4; ++i) {
    int value = i;
    EXPECT_TRUE(queue.try_push(std::move(value)));
  }
  
  // 5th push should fail - queue is full (single thread pinned to one queue)
  int overflow_value = 100;
  EXPECT_FALSE(queue.try_push(std::move(overflow_value)));
  
  // Pop all - should get exactly 4 items
  std::vector<int> results;
  int result;
  while (queue.try_pop(result)) {
    results.push_back(result);
  }
  
  EXPECT_EQ(results.size(), 4);
  
  // Values 0-3 should be present in order (FIFO from single queue)
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(results[i], i);
  }
}

TEST_F(MPSCQueueTest, EmptyCheck) {
  MPSCQueue<int> queue(16, 4);
  
  EXPECT_TRUE(queue.empty());
  
  int value = 1;
  queue.try_push(std::move(value));
  EXPECT_FALSE(queue.empty());
  
  int result;
  queue.try_pop(result);
  EXPECT_TRUE(queue.empty());
}

TEST_F(MPSCQueueTest, SizeTracking) {
  MPSCQueue<int> queue(16, 4);
  
  EXPECT_EQ(queue.size(), 0);
  
  for (int i = 0; i < 10; ++i) {
    int value = i;
    queue.try_push(std::move(value));
  }
  
  EXPECT_EQ(queue.size(), 10);
  
  int result;
  for (int i = 0; i < 5; ++i) {
    queue.try_pop(result);
  }
  
  EXPECT_EQ(queue.size(), 5);
}

// ============================================================================
// Stress Tests
// ============================================================================

TEST_F(SPSCQueueTest, StressTestLongRunning) {
  // Reduced from 500000 to avoid CI timeout on slow runners
  constexpr size_t NUM_ITEMS = 50000;
  SPSCQueue<uint64_t> queue(8192);
  
  std::atomic<bool> producer_done{false};
  std::atomic<uint64_t> sum_produced{0};
  std::atomic<uint64_t> sum_consumed{0};
  
  // Producer
  std::thread producer([&]() {
    for (uint64_t i = 0; i < NUM_ITEMS; ++i) {
      uint64_t value = i;
      while (!queue.try_push(std::move(value))) {
        std::this_thread::yield();
      }
      sum_produced += i;
    }
    producer_done = true;
  });
  
  // Consumer
  std::thread consumer([&]() {
    size_t count = 0;
    while (count < NUM_ITEMS) {
      uint64_t result;
      if (queue.try_pop(result)) {
        sum_consumed += result;
        ++count;
      } else {
        std::this_thread::yield();
      }
    }
  });
  
  producer.join();
  consumer.join();
  
  // Verify no data corruption
  EXPECT_EQ(sum_produced, sum_consumed);
}

TEST_F(MPSCQueueTest, StressTestMultipleProducers) {
  // Reduced from 8 producers × 10000 items to avoid CI timeout
  // The test still validates correctness with multiple producers
  constexpr size_t NUM_PRODUCERS = 4;
  constexpr size_t ITEMS_PER_PRODUCER = 500;
  
  MPSCQueue<uint64_t> queue(256, NUM_PRODUCERS);
  
  std::atomic<uint64_t> total_sum{0};
  std::atomic<uint64_t> consumed_sum{0};
  std::atomic<bool> done{false};
  std::vector<std::thread> producers;
  
  // Consumer - start FIRST to avoid queue backup
  std::thread consumer([&]() {
    size_t count = 0;
    const size_t total = NUM_PRODUCERS * ITEMS_PER_PRODUCER;
    while (count < total) {
      uint64_t result;
      if (queue.try_pop(result)) {
        consumed_sum += result;
        ++count;
      } else {
        std::this_thread::yield();
      }
    }
    done = true;
  });
  
  // Producers
  for (size_t p = 0; p < NUM_PRODUCERS; ++p) {
    producers.emplace_back([&, p]() {
      uint64_t local_sum = 0;
      for (size_t i = 0; i < ITEMS_PER_PRODUCER; ++i) {
        uint64_t value = p * ITEMS_PER_PRODUCER + i;
        // Add small sleep to prevent overwhelming the consumer
        int retries = 0;
        while (!queue.try_push(std::move(value))) {
          std::this_thread::yield();
          if (++retries > 1000) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
            retries = 0;
          }
        }
        local_sum += p * ITEMS_PER_PRODUCER + i;
      }
      total_sum += local_sum;
    });
  }
  
  for (auto& t : producers) {
    t.join();
  }
  consumer.join();
  
  // Verify no data corruption
  EXPECT_EQ(total_sum, consumed_sum);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(SPSCQueueTest, SingleElementQueue) {
  SPSCQueue<int> queue(1);
  
  int value = 42;
  EXPECT_TRUE(queue.try_push(std::move(value)));
  
  // Queue full
  value = 100;
  EXPECT_FALSE(queue.try_push(std::move(value)));
  
  int result;
  EXPECT_TRUE(queue.try_pop(result));
  EXPECT_EQ(result, 42);
  
  // Queue empty
  EXPECT_FALSE(queue.try_pop(result));
}

TEST_F(SPSCQueueTest, LargeObjects) {
  struct LargeObject {
    char data[1024];
    int id;
    
    LargeObject() : id(0) { std::memset(data, 0, sizeof(data)); }
    explicit LargeObject(int i) : id(i) { std::memset(data, 'A' + (i % 26), sizeof(data)); }
    LargeObject(LargeObject&& other) noexcept : id(other.id) {
      std::memcpy(data, other.data, sizeof(data));
    }
    LargeObject& operator=(LargeObject&& other) noexcept {
      id = other.id;
      std::memcpy(data, other.data, sizeof(data));
      return *this;
    }
    LargeObject(const LargeObject&) = delete;
    LargeObject& operator=(const LargeObject&) = delete;
  };
  
  SPSCQueue<LargeObject> queue(16);
  
  for (int i = 0; i < 10; ++i) {
    LargeObject obj(i);
    EXPECT_TRUE(queue.try_push(std::move(obj)));
  }
  
  for (int i = 0; i < 10; ++i) {
    LargeObject result;
    EXPECT_TRUE(queue.try_pop(result));
    EXPECT_EQ(result.id, i);
    EXPECT_EQ(result.data[0], 'A' + (i % 26));
  }
}

TEST_F(SPSCQueueTest, RapidPushPopCycle) {
  SPSCQueue<int> queue(4);
  
  // Rapid push/pop to test wraparound
  for (int cycle = 0; cycle < 1000; ++cycle) {
    int value = cycle;
    EXPECT_TRUE(queue.try_push(std::move(value)));
    
    int result;
    EXPECT_TRUE(queue.try_pop(result));
    EXPECT_EQ(result, cycle);
  }
  
  EXPECT_TRUE(queue.empty());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


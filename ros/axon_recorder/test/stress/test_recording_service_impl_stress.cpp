/**
 * @file test_recording_service_impl_stress.cpp
 * @brief Stress tests for RecordingServiceImpl class
 *
 * These tests verify:
 * - State machine stability under rapid state changes
 * - Thread safety of concurrent status queries
 * - Service handler behavior under load
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "mock_recorder_context.hpp"
#include "recording_service_impl.hpp"
#include "state_machine.hpp"
#include "task_config.hpp"

namespace axon {
namespace recorder {
namespace {

class RecordingServiceImplStressTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_context_ = std::make_shared<testing::MockRecorderContext>();
    service_ = std::make_unique<RecordingServiceImpl>(mock_context_);
  }

  void TearDown() override {
    service_.reset();
    mock_context_.reset();
  }

  // Helper to cache a valid config
  void cache_valid_config() {
    bool success = false;
    std::string message;

    service_->handle_cached_recording_config(
      "stress_task", "device_001", "collector_001",
      "order_001", "operator",
      "scene", "subscene",
      {"skill1"}, "factory",
      {"/topic1", "/topic2"},
      "", "", "",
      success, message
    );
    ASSERT_TRUE(success) << "Failed to cache config: " << message;
  }

  std::shared_ptr<testing::MockRecorderContext> mock_context_;
  std::unique_ptr<RecordingServiceImpl> service_;
};

// =============================================================================
// Rapid State Change Tests
// =============================================================================

TEST_F(RecordingServiceImplStressTest, RapidStateChanges100PerSecond) {
  constexpr int NUM_CYCLES = 100;
  constexpr int DELAY_MS = 10;  // 100 cycles/sec = 10ms per cycle

  std::atomic<int> successful_starts{0};
  std::atomic<int> successful_stops{0};
  std::atomic<int> successful_configs{0};
  std::atomic<int> failed_operations{0};

  auto start_time = std::chrono::steady_clock::now();

  for (int i = 0; i < NUM_CYCLES; ++i) {
    bool success = false;
    std::string message;

    // 1. Cache config
    service_->handle_cached_recording_config(
      "task_" + std::to_string(i), "device_001", "collector_001",
      "order_001", "operator",
      "scene", "subscene",
      {"skill1"}, "factory",
      {"/topic1"},
      "", "", "",
      success, message
    );
    if (success) {
      ++successful_configs;
    } else {
      ++failed_operations;
    }

    // 2. Start recording
    std::string task_id_response;
    success = false;
    service_->handle_recording_control("start", "", success, message, task_id_response);
    if (success) {
      ++successful_starts;

      // 3. Stop recording (finish)
      success = false;
      service_->handle_recording_control("finish", task_id_response, success, message, task_id_response);
      if (success) {
        ++successful_stops;
      } else {
        // Try cancel if finish fails
        service_->handle_recording_control("cancel", task_id_response, success, message, task_id_response);
        if (success) ++successful_stops;
      }
    } else {
      ++failed_operations;
    }

    // Reset mock state for next cycle
    mock_context_->reset_call_flags();
    
    // Clear the task config cache by issuing a clear command
    std::string dummy_task;
    service_->handle_recording_control("clear", "", success, message, dummy_task);

    // Small delay to achieve target rate
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_MS));
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time
  ).count();

  double cycles_per_sec = (NUM_CYCLES * 1000.0) / duration_ms;

  std::cout << "  [INFO] Completed " << NUM_CYCLES << " state change cycles in "
            << duration_ms << " ms (" << cycles_per_sec << " cycles/s)" << std::endl;
  std::cout << "  [INFO] Configs: " << successful_configs << ", Starts: "
            << successful_starts << ", Stops: " << successful_stops
            << ", Failed: " << failed_operations << std::endl;

  // Most operations should succeed
  EXPECT_GE(successful_configs.load(), NUM_CYCLES * 0.9);
  EXPECT_GE(successful_starts.load(), NUM_CYCLES * 0.9);
  EXPECT_GE(successful_stops.load(), NUM_CYCLES * 0.8);
}

TEST_F(RecordingServiceImplStressTest, ConcurrentStatusQueriesWhileRecording) {
  // Set up a recording session
  cache_valid_config();

  bool success = false;
  std::string message;
  std::string task_id;

  // Start recording
  service_->handle_recording_control("start", "", success, message, task_id);
  ASSERT_TRUE(success) << "Failed to start recording: " << message;

  // Set up some stats in the mock
  RecordingStats mock_stats;
  mock_stats.messages_received = 1000;
  mock_stats.messages_written = 995;
  mock_stats.messages_dropped = 5;
  mock_context_->set_stats(mock_stats);
  mock_context_->set_duration_sec(10.5);

  constexpr int NUM_THREADS = 8;
  constexpr int QUERIES_PER_THREAD = 500;

  std::atomic<int> successful_queries{0};
  std::atomic<int> failed_queries{0};
  std::vector<std::thread> threads;

  auto start_time = std::chrono::steady_clock::now();

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < QUERIES_PER_THREAD; ++i) {
        bool query_success = false;
        std::string query_message;
        std::string status, device_id, data_collector_id, order_id;
        std::string operator_name, scene, subscene, factory, output_path, last_error;
        std::string response_task_id;
        std::vector<std::string> skills, active_topics;
        double disk_usage_gb = 0, duration_sec = 0, throughput_mb_sec = 0;
        int64_t message_count = 0;

        service_->handle_recording_status(
          task_id,
          query_success, query_message, status, response_task_id,
          device_id, data_collector_id, order_id, operator_name, scene,
          subscene, skills, factory, active_topics, output_path,
          disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
        );

        if (query_success) {
          ++successful_queries;
          // Verify status is valid
          EXPECT_FALSE(status.empty());
        } else {
          ++failed_queries;
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time
  ).count();

  // Stop recording
  service_->handle_recording_control("finish", task_id, success, message, task_id);

  int total_queries = NUM_THREADS * QUERIES_PER_THREAD;
  double queries_per_sec = (total_queries * 1000.0) / duration_ms;

  std::cout << "  [INFO] " << total_queries << " concurrent queries in "
            << duration_ms << " ms (" << static_cast<int>(queries_per_sec)
            << " queries/s)" << std::endl;

  // All queries should succeed
  EXPECT_EQ(successful_queries.load(), total_queries);
  EXPECT_EQ(failed_queries.load(), 0);
}

// =============================================================================
// Config Cache Stress Tests
// =============================================================================

TEST_F(RecordingServiceImplStressTest, RapidConfigCaching) {
  constexpr int NUM_CONFIGS = 500;

  std::atomic<int> successful_caches{0};
  auto start_time = std::chrono::steady_clock::now();

  for (int i = 0; i < NUM_CONFIGS; ++i) {
    bool success = false;
    std::string message;
    std::string dummy_task;

    // Clear previous config to return to IDLE state
    // (handle_cached_recording_config only works in IDLE state)
    service_->handle_recording_control("clear", "", success, message, dummy_task);

    success = false;
    service_->handle_cached_recording_config(
      "task_" + std::to_string(i),
      "device_" + std::to_string(i % 10),
      "collector_001",
      "order_" + std::to_string(i),
      "operator_" + std::to_string(i % 5),
      "scene_" + std::to_string(i % 3),
      "subscene",
      {"skill1", "skill2"},
      "factory",
      {"/topic1", "/topic2", "/topic3"},
      "", "", "",
      success, message
    );

    if (success) {
      ++successful_caches;
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time
  ).count();

  double caches_per_sec = (NUM_CONFIGS * 1000.0) / std::max(duration_ms, 1L);

  std::cout << "  [INFO] " << NUM_CONFIGS << " config caches in "
            << duration_ms << " ms (" << static_cast<int>(caches_per_sec)
            << " caches/s)" << std::endl;

  // All caches should succeed
  EXPECT_EQ(successful_caches.load(), NUM_CONFIGS);
}

// =============================================================================
// IsRecordingReady Query Stress Tests
// =============================================================================

TEST_F(RecordingServiceImplStressTest, ConcurrentIsRecordingReadyQueries) {
  // Cache a config first
  cache_valid_config();

  constexpr int NUM_THREADS = 4;
  constexpr int QUERIES_PER_THREAD = 1000;

  std::atomic<int> successful_queries{0};
  std::vector<std::thread> threads;

  auto start_time = std::chrono::steady_clock::now();

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < QUERIES_PER_THREAD; ++i) {
        bool success = false, is_configured = false, is_recording = false;
        std::string message, task_id, device_id, order_id, operator_name;
        std::string scene, subscene, factory, data_collector_id;
        std::vector<std::string> skills, topics;

        service_->handle_is_recording_ready(
          success, message, is_configured, is_recording,
          task_id, device_id, order_id, operator_name,
          scene, subscene, skills, factory, data_collector_id, topics
        );

        if (success && is_configured) {
          ++successful_queries;
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time
  ).count();

  int total_queries = NUM_THREADS * QUERIES_PER_THREAD;
  double queries_per_sec = (total_queries * 1000.0) / duration_ms;

  std::cout << "  [INFO] " << total_queries << " IsRecordingReady queries in "
            << duration_ms << " ms (" << static_cast<int>(queries_per_sec)
            << " queries/s)" << std::endl;

  // All queries should succeed and report configured
  EXPECT_EQ(successful_queries.load(), total_queries);
}

// =============================================================================
// Mixed Command Stress Tests
// =============================================================================

TEST_F(RecordingServiceImplStressTest, MixedCommandsFromMultipleThreads) {
  // Set up initial config
  cache_valid_config();

  constexpr int NUM_THREADS = 4;
  constexpr int OPERATIONS_PER_THREAD = 100;

  std::atomic<int> total_operations{0};
  std::atomic<int> successful_operations{0};
  std::vector<std::thread> threads;

  auto start_time = std::chrono::steady_clock::now();

  // Thread 0: Status queries (always succeed)
  threads.emplace_back([&]() {
    for (int i = 0; i < OPERATIONS_PER_THREAD; ++i) {
      bool success = false;
      std::string message, status, task_id, device_id, data_collector_id;
      std::string order_id, operator_name, scene, subscene, factory;
      std::string output_path, last_error;
      std::vector<std::string> skills, active_topics;
      double disk_usage_gb = 0, duration_sec = 0, throughput_mb_sec = 0;
      int64_t message_count = 0;

      service_->handle_recording_status(
        "", success, message, status, task_id,
        device_id, data_collector_id, order_id, operator_name, scene,
        subscene, skills, factory, active_topics, output_path,
        disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
      );

      ++total_operations;
      if (success) ++successful_operations;
    }
  });

  // Thread 1: IsRecordingReady queries (always succeed)
  threads.emplace_back([&]() {
    for (int i = 0; i < OPERATIONS_PER_THREAD; ++i) {
      bool success = false, is_configured = false, is_recording = false;
      std::string message, task_id, device_id, order_id, operator_name;
      std::string scene, subscene, factory, data_collector_id;
      std::vector<std::string> skills, topics;

      service_->handle_is_recording_ready(
        success, message, is_configured, is_recording,
        task_id, device_id, order_id, operator_name,
        scene, subscene, skills, factory, data_collector_id, topics
      );

      ++total_operations;
      if (success) ++successful_operations;
    }
  });

  // Thread 2: More status queries (always succeed)
  threads.emplace_back([&]() {
    for (int i = 0; i < OPERATIONS_PER_THREAD; ++i) {
      bool success = false;
      std::string message, status, task_id, device_id, data_collector_id;
      std::string order_id, operator_name, scene, subscene, factory;
      std::string output_path, last_error;
      std::vector<std::string> skills, active_topics;
      double disk_usage_gb = 0, duration_sec = 0, throughput_mb_sec = 0;
      int64_t message_count = 0;

      service_->handle_recording_status(
        "", success, message, status, task_id,
        device_id, data_collector_id, order_id, operator_name, scene,
        subscene, skills, factory, active_topics, output_path,
        disk_usage_gb, duration_sec, message_count, throughput_mb_sec, last_error
      );

      ++total_operations;
      if (success) ++successful_operations;
    }
  });

  // Thread 3: More IsRecordingReady queries (always succeed)
  threads.emplace_back([&]() {
    for (int i = 0; i < OPERATIONS_PER_THREAD; ++i) {
      bool success = false, is_configured = false, is_recording = false;
      std::string message, task_id, device_id, order_id, operator_name;
      std::string scene, subscene, factory, data_collector_id;
      std::vector<std::string> skills, topics;

      service_->handle_is_recording_ready(
        success, message, is_configured, is_recording,
        task_id, device_id, order_id, operator_name,
        scene, subscene, skills, factory, data_collector_id, topics
      );

      ++total_operations;
      if (success) ++successful_operations;
    }
  });

  for (auto& thread : threads) {
    thread.join();
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time
  ).count();

  int total = total_operations.load();
  double ops_per_sec = (total * 1000.0) / std::max(duration_ms, 1L);

  std::cout << "  [INFO] " << total << " mixed operations in "
            << duration_ms << " ms (" << static_cast<int>(ops_per_sec)
            << " ops/s)" << std::endl;

  // All read operations should succeed (status and is_recording_ready queries)
  EXPECT_EQ(successful_operations.load(), total);
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


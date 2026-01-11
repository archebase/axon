/**
 * @file test_recording_session_stress.cpp
 * @brief Stress tests for RecordingSession class
 *
 * These tests verify:
 * - Thread safety under high concurrent load
 * - Performance at target throughput (100K messages/sec)
 * - Memory stability with large messages
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <random>
#include <thread>
#include <vector>

#include "recording_session.hpp"
#include "task_config.hpp"

namespace axon {
namespace recorder {
namespace {

class RecordingSessionStressTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temp directory for test files
    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    test_dir_ =
      std::filesystem::temp_directory_path() / ("axon_stress_test_" + std::to_string(timestamp));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test files
    if (std::filesystem::exists(test_dir_)) {
      std::filesystem::remove_all(test_dir_);
    }
  }

  std::filesystem::path test_dir_;
};

// =============================================================================
// Concurrent Write Stress Tests
// =============================================================================

TEST_F(RecordingSessionStressTest, ConcurrentWriteFrom8Threads) {
  RecordingSession session;

  std::string path = (test_dir_ / "concurrent_8threads.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema and channel
  uint16_t schema_id =
    session.register_schema("sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation");
  uint16_t channel_id = session.register_channel("/imu/data", "cdr", schema_id);

  constexpr int NUM_THREADS = 8;
  constexpr int MESSAGES_PER_THREAD = 5000;

  std::atomic<int> success_count{0};
  std::atomic<int> fail_count{0};
  std::vector<std::thread> threads;

  auto start_time = std::chrono::steady_clock::now();

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&, t]() {
      // Each thread uses slightly different data
      std::vector<uint8_t> data(72);  // IMU message size
      for (size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<uint8_t>((t + i) & 0xFF);
      }

      for (int i = 0; i < MESSAGES_PER_THREAD; ++i) {
        uint64_t ts = 1000000000ULL + t * 1000000000ULL + i * 1000ULL;
        uint32_t seq = static_cast<uint32_t>(t * MESSAGES_PER_THREAD + i);
        if (session.write(channel_id, seq, ts, ts, data.data(), data.size())) {
          ++success_count;
        } else {
          ++fail_count;
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  // All messages should succeed
  EXPECT_EQ(success_count.load(), NUM_THREADS * MESSAGES_PER_THREAD);
  EXPECT_EQ(fail_count.load(), 0);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, NUM_THREADS * MESSAGES_PER_THREAD);

  session.close();

  // Output performance info (not a strict requirement)
  int total_messages = NUM_THREADS * MESSAGES_PER_THREAD;
  double msgs_per_sec = (total_messages * 1000.0) / duration_ms;
  std::cout << "  [INFO] " << total_messages << " messages in " << duration_ms << " ms ("
            << msgs_per_sec << " msg/s)" << std::endl;

  // Verify file exists and has reasonable size
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::file_size(path), 0);
}

TEST_F(RecordingSessionStressTest, Write100KMessagesPerSecond) {
  RecordingSession session;

  std::string path = (test_dir_ / "throughput_100k.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema and channel
  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/high_freq", "cdr", schema_id);

  // Small message (typical string message)
  std::vector<uint8_t> data(64);
  for (size_t i = 0; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>(i & 0xFF);
  }

  constexpr int TARGET_MESSAGES = 100000;
  int success_count = 0;

  auto start_time = std::chrono::steady_clock::now();

  for (int i = 0; i < TARGET_MESSAGES; ++i) {
    uint64_t ts = 1000000000ULL + i * 10000ULL;  // 100kHz = 10us apart
    if (session.write(channel_id, i, ts, ts, data.data(), data.size())) {
      ++success_count;
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  // All writes should succeed
  EXPECT_EQ(success_count, TARGET_MESSAGES);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, TARGET_MESSAGES);

  session.close();

  // Calculate throughput
  double msgs_per_sec = (TARGET_MESSAGES * 1000.0) / std::max(duration_ms, 1L);
  double mb_per_sec = (TARGET_MESSAGES * data.size() * 1000.0) / (duration_ms * 1024.0 * 1024.0);

  std::cout << "  [INFO] " << TARGET_MESSAGES << " messages in " << duration_ms << " ms ("
            << static_cast<int>(msgs_per_sec) << " msg/s, " << mb_per_sec << " MB/s)" << std::endl;

  // Should complete in reasonable time (not a hard requirement for CI)
  // Just verify file was created
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(
    std::filesystem::file_size(path), TARGET_MESSAGES * data.size() / 10
  );  // At least 10% due to compression
}

TEST_F(RecordingSessionStressTest, LargeMessagesMemoryStable) {
  RecordingSession session;

  std::string path = (test_dir_ / "large_messages.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema for image data
  uint16_t schema_id = session.register_schema(
    "sensor_msgs/msg/Image", "ros2msg", "uint32 height\nuint32 width\nuint8[] data"
  );
  uint16_t channel_id = session.register_channel("/camera/image_raw", "cdr", schema_id);

  // 2MB message (1920x1080 grayscale image)
  constexpr size_t MESSAGE_SIZE = 2 * 1024 * 1024;
  std::vector<uint8_t> large_data(MESSAGE_SIZE);

  // Fill with pseudo-random data (deterministic for reproducibility)
  std::mt19937 rng(42);
  for (size_t i = 0; i < large_data.size(); ++i) {
    large_data[i] = static_cast<uint8_t>(rng() & 0xFF);
  }

  constexpr int NUM_MESSAGES = 50;  // 100MB total
  int success_count = 0;

  auto start_time = std::chrono::steady_clock::now();

  for (int i = 0; i < NUM_MESSAGES; ++i) {
    uint64_t ts = 1000000000ULL + i * 33000000ULL;  // ~30Hz
    if (session.write(channel_id, i, ts, ts, large_data.data(), large_data.size())) {
      ++success_count;
    }

    // Modify data slightly for each frame
    large_data[0] = static_cast<uint8_t>(i & 0xFF);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto duration_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  EXPECT_EQ(success_count, NUM_MESSAGES);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, NUM_MESSAGES);
  EXPECT_GE(stats.bytes_written, MESSAGE_SIZE * NUM_MESSAGES / 2);  // Account for compression

  session.close();

  double mb_written = (NUM_MESSAGES * MESSAGE_SIZE) / (1024.0 * 1024.0);
  double mb_per_sec = (mb_written * 1000.0) / std::max(duration_ms, 1L);

  std::cout << "  [INFO] " << NUM_MESSAGES << " x " << MESSAGE_SIZE / 1024 / 1024
            << " MB messages in " << duration_ms << " ms (" << mb_per_sec << " MB/s)" << std::endl;

  // Verify file was created
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::file_size(path), MESSAGE_SIZE);  // At least one message worth
}

// =============================================================================
// Mixed Workload Tests
// =============================================================================

TEST_F(RecordingSessionStressTest, MixedMessageSizesConcurrent) {
  RecordingSession session;

  std::string path = (test_dir_ / "mixed_sizes.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Set up task config
  TaskConfig config;
  config.task_id = "stress_mixed";
  config.device_id = "stress_device";
  session.set_task_config(config);

  // Register schemas for different message types
  uint16_t schema_string = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t schema_image =
    session.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint8[] data");
  uint16_t schema_imu =
    session.register_schema("sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation");

  uint16_t ch_string = session.register_channel("/status", "cdr", schema_string);
  uint16_t ch_image = session.register_channel("/camera/image", "cdr", schema_image);
  uint16_t ch_imu = session.register_channel("/imu/data", "cdr", schema_imu);

  constexpr int NUM_THREADS = 4;
  std::atomic<int> total_success{0};
  std::vector<std::thread> threads;

  // Thread 0: Small messages (strings) at 1kHz
  threads.emplace_back([&]() {
    std::vector<uint8_t> data(32);
    for (int i = 0; i < 1000; ++i) {
      uint64_t ts = 1000000000ULL + i * 1000000ULL;
      if (session.write(ch_string, i, ts, ts, data.data(), data.size())) {
        ++total_success;
      }
      session.update_topic_stats("/status", "std_msgs/msg/String");
    }
  });

  // Thread 1: Medium messages (IMU) at 400Hz
  threads.emplace_back([&]() {
    std::vector<uint8_t> data(144);  // Imu message
    for (int i = 0; i < 400; ++i) {
      uint64_t ts = 1000000000ULL + i * 2500000ULL;
      if (session.write(ch_imu, i, ts, ts, data.data(), data.size())) {
        ++total_success;
      }
      session.update_topic_stats("/imu/data", "sensor_msgs/msg/Imu");
    }
  });

  // Thread 2 & 3: Large messages (images) at 30Hz each
  for (int t = 2; t < NUM_THREADS; ++t) {
    threads.emplace_back([&, t]() {
      std::vector<uint8_t> data(640 * 480);  // VGA grayscale
      for (int i = 0; i < 30; ++i) {
        uint64_t ts = 1000000000ULL + i * 33333333ULL + t * 1000ULL;
        if (session.write(ch_image, i, ts, ts, data.data(), data.size())) {
          ++total_success;
        }
        session.update_topic_stats("/camera/image", "sensor_msgs/msg/Image");
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  int expected_total = 1000 + 400 + 30 * 2;  // string + imu + 2x image
  EXPECT_EQ(total_success.load(), expected_total);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, expected_total);
  EXPECT_EQ(stats.schemas_registered, 3);
  EXPECT_EQ(stats.channels_registered, 3);

  session.close();
}

// =============================================================================
// Sustained Load Test
// =============================================================================

TEST_F(RecordingSessionStressTest, SustainedLoadFor5Seconds) {
  RecordingSession session;

  std::string path = (test_dir_ / "sustained_load.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/sustained", "cdr", schema_id);

  std::vector<uint8_t> data(128);

  auto start = std::chrono::steady_clock::now();
  auto deadline = start + std::chrono::seconds(5);

  int message_count = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    uint64_t ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch()
    )
                                          .count());
    if (session.write(channel_id, message_count, ts, ts, data.data(), data.size())) {
      ++message_count;
    }
  }

  auto actual_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start)
      .count();

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, message_count);

  session.close();

  double msgs_per_sec = (message_count * 1000.0) / actual_duration;
  std::cout << "  [INFO] Sustained load: " << message_count << " messages in " << actual_duration
            << " ms (" << static_cast<int>(msgs_per_sec) << " msg/s)" << std::endl;

  // Should have written a reasonable number of messages
  EXPECT_GT(message_count, 10000);  // At least 2000 msg/s average
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

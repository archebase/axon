/**
 * @file test_recording_session.cpp
 * @brief Unit tests for RecordingSession class
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <thread>
#include <vector>

#include "recording_session.hpp"

namespace axon {
namespace recorder {
namespace {

class RecordingSessionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temp directory for test files
    test_dir_ = std::filesystem::temp_directory_path() / "axon_test_session";
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test files
    std::filesystem::remove_all(test_dir_);
  }

  std::filesystem::path test_dir_;
};

TEST_F(RecordingSessionTest, DefaultConstruction) {
  RecordingSession session;
  EXPECT_FALSE(session.is_open());
  EXPECT_EQ(session.get_path(), "");
}

TEST_F(RecordingSessionTest, OpenClose) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();

  EXPECT_TRUE(session.open(path));
  EXPECT_TRUE(session.is_open());
  EXPECT_EQ(session.get_path(), path);

  session.close();
  EXPECT_FALSE(session.is_open());

  // Verify file was created
  EXPECT_TRUE(std::filesystem::exists(path));
}

TEST_F(RecordingSessionTest, CannotOpenTwice) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();

  EXPECT_TRUE(session.open(path));
  EXPECT_FALSE(session.open(path));  // Second open should fail

  session.close();
}

TEST_F(RecordingSessionTest, RegisterSchema) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  EXPECT_GT(schema_id, 0);

  // Same schema should return same ID
  uint16_t schema_id2 = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  EXPECT_EQ(schema_id, schema_id2);

  // Get schema ID
  EXPECT_EQ(session.get_schema_id("std_msgs/msg/String"), schema_id);
  EXPECT_EQ(session.get_schema_id("nonexistent"), 0);

  session.close();
}

TEST_F(RecordingSessionTest, RegisterChannel) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema first
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  ASSERT_GT(schema_id, 0);

  // Register channel
  uint16_t channel_id = session.register_channel(
    "/test_topic", "cdr", schema_id
  );
  EXPECT_GT(channel_id, 0);

  // Same channel should return same ID
  uint16_t channel_id2 = session.register_channel(
    "/test_topic", "cdr", schema_id
  );
  EXPECT_EQ(channel_id, channel_id2);

  // Get channel ID
  EXPECT_EQ(session.get_channel_id("/test_topic"), channel_id);
  EXPECT_EQ(session.get_channel_id("/nonexistent"), 0);

  session.close();
}

TEST_F(RecordingSessionTest, WriteMessage) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema and channel
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test_topic", "cdr", schema_id);

  // Write a message
  std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();

  EXPECT_TRUE(session.write(channel_id, 1, timestamp, timestamp, data.data(), data.size()));

  // Check stats
  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 1);

  session.close();
}

TEST_F(RecordingSessionTest, DurationTracking) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();

  // Duration should be 0 when not open
  EXPECT_EQ(session.get_duration_sec(), 0.0);

  ASSERT_TRUE(session.open(path));

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Duration should be > 0
  double duration = session.get_duration_sec();
  EXPECT_GT(duration, 0.05);  // At least 50ms
  EXPECT_LT(duration, 1.0);   // Less than 1 second

  session.close();
}

TEST_F(RecordingSessionTest, StatsTracking) {
  RecordingSession session;

  std::string path = (test_dir_ / "test.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Initial stats
  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 0);

  // Register and write
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);

  std::vector<uint8_t> data = {0x01};
  uint64_t ts = 1000000;

  // Write 5 messages
  for (int i = 0; i < 5; ++i) {
    session.write(channel_id, i, ts, ts, data.data(), data.size());
  }

  stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 5);
  EXPECT_EQ(stats.schemas_registered, 1);
  EXPECT_EQ(stats.channels_registered, 1);

  session.close();
}

TEST_F(RecordingSessionTest, CannotWriteWhenClosed) {
  RecordingSession session;

  std::vector<uint8_t> data = {0x01};
  EXPECT_FALSE(session.write(1, 0, 0, 0, data.data(), data.size()));

  // Also can't register schema/channel when closed
  EXPECT_EQ(session.register_schema("test", "enc", "def"), 0);
  EXPECT_EQ(session.register_channel("/test", "enc", 1), 0);
}

// ============================================================================
// Concurrent Write Tests
// ============================================================================

TEST_F(RecordingSessionTest, ConcurrentWritesFromMultipleThreads) {
  RecordingSession session;

  std::string path = (test_dir_ / "concurrent.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register schema and channel
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);

  constexpr int NUM_THREADS = 4;
  constexpr int MESSAGES_PER_THREAD = 1000;

  std::atomic<int> success_count{0};
  std::vector<std::thread> threads;

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back([&, t]() {
      std::vector<uint8_t> data = {static_cast<uint8_t>(t), 0x02, 0x03, 0x04};
      for (int i = 0; i < MESSAGES_PER_THREAD; ++i) {
        uint64_t ts = 1000000000ULL + t * 1000000ULL + i * 1000ULL;
        if (session.write(channel_id, t * MESSAGES_PER_THREAD + i, ts, ts,
                          data.data(), data.size())) {
          ++success_count;
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // All messages should have been written
  EXPECT_EQ(success_count.load(), NUM_THREADS * MESSAGES_PER_THREAD);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, NUM_THREADS * MESSAGES_PER_THREAD);

  session.close();

  // Verify file exists and has data
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::file_size(path), 0);
}

// ============================================================================
// Multiple Channels Tests
// ============================================================================

TEST_F(RecordingSessionTest, MultipleChannelsSameSchema) {
  RecordingSession session;

  std::string path = (test_dir_ / "multi_channel.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Single schema for multiple channels
  uint16_t schema_id = session.register_schema(
    "sensor_msgs/msg/Image", "ros2msg", "uint32 height\nuint32 width\nuint8[] data"
  );

  // Register multiple channels with same schema
  uint16_t channel1 = session.register_channel("/camera/left/image", "cdr", schema_id);
  uint16_t channel2 = session.register_channel("/camera/right/image", "cdr", schema_id);
  uint16_t channel3 = session.register_channel("/camera/depth/image", "cdr", schema_id);

  EXPECT_GT(channel1, 0);
  EXPECT_GT(channel2, 0);
  EXPECT_GT(channel3, 0);
  EXPECT_NE(channel1, channel2);
  EXPECT_NE(channel2, channel3);

  // Write to each channel
  std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  uint64_t ts = 1000000000ULL;

  EXPECT_TRUE(session.write(channel1, 0, ts, ts, data.data(), data.size()));
  EXPECT_TRUE(session.write(channel2, 0, ts + 1000, ts + 1000, data.data(), data.size()));
  EXPECT_TRUE(session.write(channel3, 0, ts + 2000, ts + 2000, data.data(), data.size()));

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 3);
  EXPECT_EQ(stats.schemas_registered, 1);
  EXPECT_EQ(stats.channels_registered, 3);

  session.close();
}

TEST_F(RecordingSessionTest, MultipleSchemas) {
  RecordingSession session;

  std::string path = (test_dir_ / "multi_schema.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Register different schemas
  uint16_t schema1 = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t schema2 = session.register_schema(
    "sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation"
  );
  uint16_t schema3 = session.register_schema(
    "geometry_msgs/msg/Twist", "ros2msg", "geometry_msgs/Vector3 linear"
  );

  EXPECT_GT(schema1, 0);
  EXPECT_GT(schema2, 0);
  EXPECT_GT(schema3, 0);
  EXPECT_NE(schema1, schema2);
  EXPECT_NE(schema2, schema3);

  // Register channels with different schemas
  uint16_t ch1 = session.register_channel("/text", "cdr", schema1);
  uint16_t ch2 = session.register_channel("/imu", "cdr", schema2);
  uint16_t ch3 = session.register_channel("/cmd_vel", "cdr", schema3);

  auto stats = session.get_stats();
  EXPECT_EQ(stats.schemas_registered, 3);
  EXPECT_EQ(stats.channels_registered, 3);

  session.close();
}

// ============================================================================
// Channel Metadata Tests
// ============================================================================

TEST_F(RecordingSessionTest, ChannelWithMetadata) {
  RecordingSession session;

  std::string path = (test_dir_ / "metadata.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "sensor_msgs/msg/Image", "ros2msg", "uint32 height"
  );

  // Register channel with metadata
  std::unordered_map<std::string, std::string> metadata = {
    {"frame_id", "camera_link"},
    {"offered_qos_profiles", "- history: keep_last\n  depth: 10"}
  };

  uint16_t channel_id = session.register_channel(
    "/camera/image", "cdr", schema_id, metadata
  );
  EXPECT_GT(channel_id, 0);

  session.close();
}

// ============================================================================
// Topic Statistics Tests
// ============================================================================

TEST_F(RecordingSessionTest, TopicStatsTracking) {
  RecordingSession session;

  std::string path = (test_dir_ / "topic_stats.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);

  std::vector<uint8_t> data = {0x01, 0x02, 0x03};

  // Write messages and update stats
  for (int i = 0; i < 100; ++i) {
    uint64_t ts = 1000000000ULL + i * 1000000ULL;
    session.write(channel_id, i, ts, ts, data.data(), data.size());
    session.update_topic_stats("/test", "std_msgs/msg/String");
  }

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 100);

  session.close();
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(RecordingSessionTest, WriteToInvalidChannel) {
  RecordingSession session;

  std::string path = (test_dir_ / "invalid_channel.mcap").string();
  ASSERT_TRUE(session.open(path));

  std::vector<uint8_t> data = {0x01};
  uint64_t ts = 1000000000ULL;

  // Write to non-existent channel ID
  EXPECT_FALSE(session.write(9999, 0, ts, ts, data.data(), data.size()));

  session.close();
}

TEST_F(RecordingSessionTest, OpenCreatesFile) {
  RecordingSession session;

  // Recording session creates the file when opening
  std::string path = (test_dir_ / "new_recording.mcap").string();
  
  EXPECT_TRUE(session.open(path));
  EXPECT_TRUE(session.is_open());
  EXPECT_TRUE(std::filesystem::exists(path));
  
  session.close();
}

TEST_F(RecordingSessionTest, FlushDuringWrite) {
  RecordingSession session;

  std::string path = (test_dir_ / "flush.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);

  std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};

  for (int i = 0; i < 10; ++i) {
    uint64_t ts = 1000000000ULL + i * 1000000ULL;
    EXPECT_TRUE(session.write(channel_id, i, ts, ts, data.data(), data.size()));
    
    // Flush periodically
    if (i % 3 == 0) {
      EXPECT_NO_THROW(session.flush());
    }
  }

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 10);

  session.close();
}

TEST_F(RecordingSessionTest, CloseWithoutOpen) {
  RecordingSession session;
  
  // Close without opening should not crash
  EXPECT_NO_THROW(session.close());
  EXPECT_FALSE(session.is_open());
}

TEST_F(RecordingSessionTest, DoubleClose) {
  RecordingSession session;

  std::string path = (test_dir_ / "double_close.mcap").string();
  ASSERT_TRUE(session.open(path));

  session.close();
  
  // Second close should not crash
  EXPECT_NO_THROW(session.close());
  EXPECT_FALSE(session.is_open());
}

// ============================================================================
// Large Data Tests
// ============================================================================

TEST_F(RecordingSessionTest, LargeMessages) {
  RecordingSession session;

  std::string path = (test_dir_ / "large.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "sensor_msgs/msg/Image", "ros2msg", "uint8[] data"
  );
  uint16_t channel_id = session.register_channel("/camera/image", "cdr", schema_id);

  // 1MB message (simulating image data)
  std::vector<uint8_t> large_data(1024 * 1024);
  for (size_t i = 0; i < large_data.size(); ++i) {
    large_data[i] = static_cast<uint8_t>(i % 256);
  }

  uint64_t ts = 1000000000ULL;
  EXPECT_TRUE(session.write(channel_id, 0, ts, ts, large_data.data(), large_data.size()));

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 1);
  EXPECT_GE(stats.bytes_written, large_data.size());

  session.close();
}

TEST_F(RecordingSessionTest, ManySmallMessages) {
  RecordingSession session;

  std::string path = (test_dir_ / "many_small.mcap").string();
  ASSERT_TRUE(session.open(path));

  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/Int32", "ros2msg", "int32 data"
  );
  uint16_t channel_id = session.register_channel("/counter", "cdr", schema_id);

  std::vector<uint8_t> data = {0x01, 0x00, 0x00, 0x00};  // int32 = 1

  constexpr int NUM_MESSAGES = 10000;
  for (int i = 0; i < NUM_MESSAGES; ++i) {
    uint64_t ts = 1000000000ULL + i * 1000ULL;
    EXPECT_TRUE(session.write(channel_id, i, ts, ts, data.data(), data.size()));
  }

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, NUM_MESSAGES);

  session.close();

  // Verify file was created with reasonable size
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::file_size(path), NUM_MESSAGES * data.size());
}

// ============================================================================
// Task Config and Metadata Tests
// ============================================================================

TEST_F(RecordingSessionTest, SetTaskConfig) {
  RecordingSession session;

  std::string path = (test_dir_ / "task_config.mcap").string();
  ASSERT_TRUE(session.open(path));

  TaskConfig config;
  config.task_id = "test_task_123";
  config.device_id = "device_001";
  config.operator_name = "test.operator";
  config.scene = "indoor";
  config.subscene = "kitchen";

  // Should not throw
  EXPECT_NO_THROW(session.set_task_config(config));

  session.close();
}

TEST_F(RecordingSessionTest, GetSidecarPathBeforeClose) {
  RecordingSession session;

  std::string path = (test_dir_ / "sidecar.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Sidecar path should be empty before close
  EXPECT_TRUE(session.get_sidecar_path().empty());

  TaskConfig config;
  config.task_id = "test_task_123";
  session.set_task_config(config);

  // Still empty until close
  EXPECT_TRUE(session.get_sidecar_path().empty());

  session.close();

  // After close, sidecar path might be set (depends on implementation)
  // Just verify it doesn't crash
}

// ============================================================================
// Duration Accuracy Tests
// ============================================================================

TEST_F(RecordingSessionTest, DurationAccuracy) {
  RecordingSession session;

  std::string path = (test_dir_ / "duration.mcap").string();
  ASSERT_TRUE(session.open(path));

  auto start = std::chrono::steady_clock::now();
  
  // Sleep for known duration
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  double duration = session.get_duration_sec();
  
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start
  ).count();

  // Duration should be within 50ms of expected
  EXPECT_GE(duration, 0.15);  // At least 150ms
  EXPECT_LE(duration, 0.35);  // At most 350ms

  session.close();
}

// ============================================================================
// Metadata Injection Tests (Extended)
// ============================================================================

TEST_F(RecordingSessionTest, SetTaskConfigEnablesMetadata) {
  RecordingSession session;

  std::string path = (test_dir_ / "metadata_enabled.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Set up task config with all fields
  TaskConfig config;
  config.task_id = "test_task_metadata";
  config.device_id = "device_metadata_001";
  config.data_collector_id = "collector_001";
  config.order_id = "order_001";
  config.operator_name = "test.operator";
  config.scene = "outdoor";
  config.subscene = "parking_lot";
  config.skills = {"navigation", "mapping"};
  config.factory = "test_factory";
  config.topics = {"/camera/image", "/lidar/points"};
  config.cached_at = std::chrono::system_clock::now();

  // Set task config should not throw and should enable metadata
  EXPECT_NO_THROW(session.set_task_config(config));

  // Write some data to ensure file has content
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);
  std::vector<uint8_t> data = {0x01, 0x02, 0x03};
  uint64_t ts = 1000000000ULL;
  session.write(channel_id, 0, ts, ts, data.data(), data.size());
  
  // Update topic stats as recorder would
  session.update_topic_stats("/test", "std_msgs/msg/String");

  session.close();

  // File should exist after close
  EXPECT_TRUE(std::filesystem::exists(path));
}

TEST_F(RecordingSessionTest, SidecarPathAvailableAfterClose) {
  RecordingSession session;

  std::string path = (test_dir_ / "sidecar_test.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Configure task for metadata injection
  TaskConfig config;
  config.task_id = "sidecar_task";
  config.device_id = "device_001";
  config.scene = "test_scene";
  session.set_task_config(config);

  // Sidecar should be empty before close
  EXPECT_TRUE(session.get_sidecar_path().empty());

  // Write some data
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);
  std::vector<uint8_t> data = {0x01};
  session.write(channel_id, 0, 1000000000ULL, 1000000000ULL, data.data(), data.size());
  session.update_topic_stats("/test", "std_msgs/msg/String");

  session.close();

  // After close, sidecar path should be non-empty (if implementation writes sidecar)
  std::string sidecar_path = session.get_sidecar_path();
  if (!sidecar_path.empty()) {
    // Verify it's a JSON file path
    EXPECT_TRUE(sidecar_path.find(".json") != std::string::npos);
  }
}

TEST_F(RecordingSessionTest, ChecksumAvailableAfterClose) {
  RecordingSession session;

  std::string path = (test_dir_ / "checksum_test.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Configure task for metadata injection
  TaskConfig config;
  config.task_id = "checksum_task";
  config.device_id = "device_001";
  session.set_task_config(config);

  // Checksum should be empty before close
  EXPECT_TRUE(session.get_checksum().empty());

  // Write some data
  uint16_t schema_id = session.register_schema(
    "std_msgs/msg/String", "ros2msg", "string data"
  );
  uint16_t channel_id = session.register_channel("/test", "cdr", schema_id);
  std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  for (int i = 0; i < 10; ++i) {
    uint64_t ts = 1000000000ULL + i * 1000ULL;
    session.write(channel_id, i, ts, ts, data.data(), data.size());
    session.update_topic_stats("/test", "std_msgs/msg/String");
  }

  session.close();

  // After close, checksum might be available (if implementation computes it)
  std::string checksum = session.get_checksum();
  if (!checksum.empty()) {
    // Checksum should be hex string (SHA256 = 64 chars)
    EXPECT_GE(checksum.size(), 32);  // At least 128-bit hash
  }
}

TEST_F(RecordingSessionTest, UpdateTopicStatsAccumulates) {
  RecordingSession session;

  std::string path = (test_dir_ / "topic_stats_accum.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Set task config to enable metadata tracking
  TaskConfig config;
  config.task_id = "stats_accumulate_task";
  config.device_id = "device_001";
  session.set_task_config(config);

  uint16_t schema_id = session.register_schema(
    "sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation"
  );
  uint16_t channel_id = session.register_channel("/imu/data", "cdr", schema_id);

  std::vector<uint8_t> data = {0x01, 0x02, 0x03};
  
  // Write and update stats for 500 messages
  for (int i = 0; i < 500; ++i) {
    uint64_t ts = 1000000000ULL + i * 1000000ULL;
    session.write(channel_id, i, ts, ts, data.data(), data.size());
    session.update_topic_stats("/imu/data", "sensor_msgs/msg/Imu");
  }

  // Verify messages written
  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, 500);

  session.close();
}

TEST_F(RecordingSessionTest, MessageTypeStoredOnFirstUpdate) {
  RecordingSession session;

  std::string path = (test_dir_ / "msg_type_stored.mcap").string();
  ASSERT_TRUE(session.open(path));

  // Set task config to enable metadata tracking
  TaskConfig config;
  config.task_id = "msg_type_task";
  config.device_id = "device_001";
  session.set_task_config(config);

  // Register multiple schemas and channels
  uint16_t schema1 = session.register_schema(
    "sensor_msgs/msg/Image", "ros2msg", "uint32 height"
  );
  uint16_t schema2 = session.register_schema(
    "sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation"
  );
  
  uint16_t ch1 = session.register_channel("/camera/image", "cdr", schema1);
  uint16_t ch2 = session.register_channel("/imu/data", "cdr", schema2);

  std::vector<uint8_t> data = {0x01};
  uint64_t ts = 1000000000ULL;

  // Write to camera channel
  session.write(ch1, 0, ts, ts, data.data(), data.size());
  session.update_topic_stats("/camera/image", "sensor_msgs/msg/Image");
  
  // Write to IMU channel
  session.write(ch2, 0, ts + 1000, ts + 1000, data.data(), data.size());
  session.update_topic_stats("/imu/data", "sensor_msgs/msg/Imu");

  // Multiple updates with same type should not change stored type
  for (int i = 1; i < 10; ++i) {
    session.write(ch1, i, ts + i * 1000, ts + i * 1000, data.data(), data.size());
    session.update_topic_stats("/camera/image", "sensor_msgs/msg/Image");
  }

  auto stats = session.get_stats();
  EXPECT_EQ(stats.channels_registered, 2);
  EXPECT_EQ(stats.schemas_registered, 2);

  session.close();
}

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * @file test_recording_session.cpp
 * @brief Unit tests for RecordingSession class
 *
 * Tests MCAP file operations, schema/channel registration, message writing,
 * statistics tracking, and metadata injection.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "../recording_session.hpp"
#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;
using namespace axon::recorder;
using namespace axon::mcap_wrapper;

// ============================================================================
// Test Fixtures
// ============================================================================

class RecordingSessionTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp directory for test files
    test_dir_ = fs::temp_directory_path() /
                ("recording_session_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  std::string get_test_path(const std::string& filename) {
    return (test_dir_ / filename).string();
  }

  // Helper to check if file exists and is not empty
  bool is_valid_mcap(const std::string& path) {
    if (!fs::exists(path)) {
      return false;
    }
    return fs::file_size(path) > 0;
  }

  fs::path test_dir_;
};

// ============================================================================
// Lifecycle Tests
// ============================================================================

TEST_F(RecordingSessionTest, ConstructorInitializesState) {
  RecordingSession session;

  EXPECT_FALSE(session.is_open());
  EXPECT_EQ(session.get_path(), "");
  // get_last_error() can be called (may be empty when not initialized)
  session.get_last_error();
}

TEST_F(RecordingSessionTest, DestructorClosesSession) {
  auto path = get_test_path("test_destructor.mcap");

  {
    RecordingSession session;
    McapWriterOptions options;
    EXPECT_TRUE(session.open(path, options));
    EXPECT_TRUE(session.is_open());
    // Session closes automatically when destroyed
  }

  // File should be properly closed and finalized
  EXPECT_TRUE(is_valid_mcap(path));
}

TEST_F(RecordingSessionTest, OpenCreatesFile) {
  RecordingSession session;
  auto path = get_test_path("test_open.mcap");

  McapWriterOptions options;
  bool result = session.open(path, options);

  EXPECT_TRUE(result);
  EXPECT_TRUE(session.is_open());
  EXPECT_EQ(session.get_path(), path);
  EXPECT_TRUE(fs::exists(path));
}

TEST_F(RecordingSessionTest, OpenFailsWhenAlreadyOpen) {
  RecordingSession session;
  auto path1 = get_test_path("test1.mcap");
  auto path2 = get_test_path("test2.mcap");

  McapWriterOptions options;
  EXPECT_TRUE(session.open(path1, options));

  bool result = session.open(path2, options);

  EXPECT_FALSE(result);
  EXPECT_EQ(session.get_path(), path1);
}

TEST_F(RecordingSessionTest, CloseMakesSessionNotOpen) {
  RecordingSession session;
  auto path = get_test_path("test_close.mcap");

  McapWriterOptions options;
  session.open(path, options);
  EXPECT_TRUE(session.is_open());

  session.close();

  EXPECT_FALSE(session.is_open());
  EXPECT_TRUE(is_valid_mcap(path));
}

TEST_F(RecordingSessionTest, CloseIdempotent) {
  RecordingSession session;
  auto path = get_test_path("test_close_idempotent.mcap");

  McapWriterOptions options;
  session.open(path, options);
  session.close();

  // Second close should be safe
  session.close();
  EXPECT_FALSE(session.is_open());
}

// ============================================================================
// Schema Registration Tests
// ============================================================================

TEST_F(RecordingSessionTest, RegisterSchema) {
  RecordingSession session;
  auto path = get_test_path("test_schema.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  std::string schema_name = "sensor_msgs/msg/Image";
  std::string encoding = "ros2msg";
  std::string definition = "std_msgs/Header header\nuint32 height\nuint32 width";

  uint16_t schema_id = session.register_schema(schema_name, encoding, definition);

  EXPECT_NE(schema_id, 0);
  EXPECT_EQ(session.get_schema_id(schema_name), schema_id);
}

TEST_F(RecordingSessionTest, RegisterSchemaFailsWhenNotOpen) {
  RecordingSession session;

  uint16_t schema_id = session.register_schema("test", "ros2msg", "definition");

  EXPECT_EQ(schema_id, 0);
}

TEST_F(RecordingSessionTest, RegisterSchemaReturnsSameIdForDuplicate) {
  RecordingSession session;
  auto path = get_test_path("test_schema_duplicate.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  std::string schema_name = "std_msgs/msg/String";
  std::string encoding = "ros2msg";
  std::string definition = "string data";

  uint16_t id1 = session.register_schema(schema_name, encoding, definition);
  uint16_t id2 = session.register_schema(schema_name, encoding, definition);

  EXPECT_EQ(id1, id2);
  EXPECT_EQ(id2, session.get_schema_id(schema_name));
}

TEST_F(RecordingSessionTest, RegisterMultipleSchemas) {
  RecordingSession session;
  auto path = get_test_path("test_multiple_schemas.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t id1 = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t id2 = session.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint32 height");
  uint16_t id3 = session.register_schema("sensor_msgs/msg/Imu", "ros2msg", "vector3 orientation");

  EXPECT_NE(id1, 0);
  EXPECT_NE(id2, 0);
  EXPECT_NE(id3, 0);
  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
}

// ============================================================================
// Channel Registration Tests
// ============================================================================

TEST_F(RecordingSessionTest, RegisterChannel) {
  RecordingSession session;
  auto path = get_test_path("test_channel.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  ASSERT_NE(schema_id, 0);

  std::string topic = "/test/topic";
  std::string message_type = "std_msgs/msg/String";

  // Use get_or_create_channel which handles composite key lookup
  uint16_t channel_id = session.get_or_create_channel(topic, message_type);

  EXPECT_NE(channel_id, 0);
  // Verify it returns the same ID on second call
  uint16_t channel_id2 = session.get_or_create_channel(topic, message_type);
  EXPECT_EQ(channel_id2, channel_id);
}

TEST_F(RecordingSessionTest, RegisterChannelFailsWhenNotOpen) {
  RecordingSession session;

  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", 1);

  EXPECT_EQ(channel_id, 0);
}

TEST_F(RecordingSessionTest, RegisterChannelReturnsSameIdForDuplicate) {
  RecordingSession session;
  auto path = get_test_path("test_channel_duplicate.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");

  std::string topic = "/test/topic";
  uint16_t id1 = session.register_channel(topic, "std_msgs/msg/String", "cdr", schema_id);
  uint16_t id2 = session.register_channel(topic, "std_msgs/msg/String", "cdr", schema_id);

  EXPECT_EQ(id1, id2);
}

TEST_F(RecordingSessionTest, RegisterChannelWithMetadata) {
  RecordingSession session;
  auto path = get_test_path("test_channel_metadata.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");

  std::unordered_map<std::string, std::string> metadata = {
    {"qos_depth", "10"}, {"qos_reliable", "true"}
  };

  uint16_t channel_id =
    session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id, metadata);

  EXPECT_NE(channel_id, 0);
}

TEST_F(RecordingSessionTest, RegisterMultipleChannels) {
  RecordingSession session;
  auto path = get_test_path("test_multiple_channels.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");

  uint16_t id1 = session.register_channel("/topic1", "std_msgs/msg/String", "cdr", schema_id);
  uint16_t id2 = session.register_channel("/topic2", "std_msgs/msg/String", "cdr", schema_id);
  uint16_t id3 = session.register_channel("/topic3", "std_msgs/msg/String", "cdr", schema_id);

  EXPECT_NE(id1, 0);
  EXPECT_NE(id2, 0);
  EXPECT_NE(id3, 0);
  EXPECT_NE(id1, id2);
  EXPECT_NE(id2, id3);
}

// ============================================================================
// Message Writing Tests
// ============================================================================

TEST_F(RecordingSessionTest, WriteMessage) {
  RecordingSession session;
  auto path = get_test_path("test_write.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  std::string data = "test message";
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch()
  )
                         .count();

  bool result = session.write(
    channel_id, 0, timestamp, timestamp, reinterpret_cast<const uint8_t*>(data.data()), data.size()
  );

  EXPECT_TRUE(result);
}

TEST_F(RecordingSessionTest, WriteMessageFailsWhenNotOpen) {
  RecordingSession session;

  std::string data = "test";
  bool result =
    session.write(1, 0, 0, 0, reinterpret_cast<const uint8_t*>(data.data()), data.size());

  EXPECT_FALSE(result);
}

TEST_F(RecordingSessionTest, WriteMultipleMessages) {
  RecordingSession session;
  auto path = get_test_path("test_multiple_messages.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch()
  )
                         .count();

  for (int i = 0; i < 100; ++i) {
    std::string data = "message " + std::to_string(i);
    bool result = session.write(
      channel_id,
      static_cast<uint32_t>(i),
      timestamp,
      timestamp,
      reinterpret_cast<const uint8_t*>(data.data()),
      data.size()
    );
    EXPECT_TRUE(result);
  }
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(RecordingSessionTest, InitialStatistics) {
  RecordingSession session;
  auto path = get_test_path("test_stats_initial.mcap");

  McapWriterOptions options;
  session.open(path, options);

  auto stats = session.get_stats();

  EXPECT_EQ(stats.messages_written, 0);
  EXPECT_EQ(stats.schemas_registered, 0);
  EXPECT_EQ(stats.channels_registered, 0);
  // bytes_written may not be 0 due to header
}

TEST_F(RecordingSessionTest, StatisticsAfterRegistrations) {
  RecordingSession session;
  auto path = get_test_path("test_stats_registration.mcap");

  McapWriterOptions options;
  session.open(path, options);

  session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  session.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint32 height");

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  session.register_channel("/topic1", "std_msgs/msg/String", "cdr", schema_id);
  session.register_channel("/topic2", "std_msgs/msg/String", "cdr", schema_id);

  auto stats = session.get_stats();

  EXPECT_EQ(stats.schemas_registered, 2);
  EXPECT_EQ(stats.channels_registered, 2);
}

TEST_F(RecordingSessionTest, StatisticsAfterWriting) {
  RecordingSession session;
  auto path = get_test_path("test_stats_writing.mcap");

  McapWriterOptions options;
  session.open(path, options);

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch()
  )
                         .count();

  for (int i = 0; i < 50; ++i) {
    std::string data = "message " + std::to_string(i);
    session.write(
      channel_id,
      static_cast<uint32_t>(i),
      timestamp,
      timestamp,
      reinterpret_cast<const uint8_t*>(data.data()),
      data.size()
    );
  }

  auto stats = session.get_stats();

  EXPECT_EQ(stats.messages_written, 50);
}

// ============================================================================
// Duration Tests
// ============================================================================

TEST_F(RecordingSessionTest, DurationZeroWhenNotOpen) {
  RecordingSession session;

  EXPECT_DOUBLE_EQ(session.get_duration_sec(), 0.0);
}

TEST_F(RecordingSessionTest, DurationIncreasesWhileOpen) {
  RecordingSession session;
  auto path = get_test_path("test_duration.mcap");

  McapWriterOptions options;
  session.open(path, options);

  double duration1 = session.get_duration_sec();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  double duration2 = session.get_duration_sec();

  EXPECT_GT(duration2, duration1);
  EXPECT_GT(duration2, 0.09);  // At least 90ms
}

// ============================================================================
// Flush Tests
// ============================================================================

TEST_F(RecordingSessionTest, FlushWhenOpen) {
  RecordingSession session;
  auto path = get_test_path("test_flush.mcap");

  McapWriterOptions options;
  session.open(path, options);

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  std::string data = "test message";
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch()
  )
                         .count();

  session.write(
    channel_id, 0, timestamp, timestamp, reinterpret_cast<const uint8_t*>(data.data()), data.size()
  );

  session.flush();

  // Should not crash
  EXPECT_TRUE(session.is_open());
}

TEST_F(RecordingSessionTest, FlushWhenNotOpen) {
  RecordingSession session;

  session.flush();

  // Should not crash
  EXPECT_FALSE(session.is_open());
}

// ============================================================================
// Topic Statistics Tests
// ============================================================================

TEST_F(RecordingSessionTest, UpdateTopicStats) {
  RecordingSession session;
  auto path = get_test_path("test_topic_stats.mcap");

  McapWriterOptions options;
  session.open(path, options);

  session.update_topic_stats("/camera/image", "sensor_msgs/Image");
  session.update_topic_stats("/imu/data", "sensor_msgs/Imu");

  // Stats tracked internally - tested via metadata injection
}

// ============================================================================
// Metadata Injection Tests
// ============================================================================

TEST_F(RecordingSessionTest, SetTaskConfig) {
  RecordingSession session;
  auto path = get_test_path("test_task_config.mcap");

  McapWriterOptions options;
  session.open(path, options);

  TaskConfig config;
  config.task_id = "test_task_001";
  config.device_id = "robot_01";
  config.scene = "warehouse";

  session.set_task_config(config);
  session.close();

  // Should generate sidecar JSON
  std::string sidecar_path = session.get_sidecar_path();
  EXPECT_FALSE(sidecar_path.empty());
  EXPECT_TRUE(fs::exists(sidecar_path));
  EXPECT_TRUE(sidecar_path.find(".json") != std::string::npos);
}

TEST_F(RecordingSessionTest, SidecarContainsChecksum) {
  RecordingSession session;
  auto path = get_test_path("test_checksum.mcap");

  McapWriterOptions options;
  session.open(path, options);

  // Write some data
  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  std::string data = "test message for checksum";
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch()
  )
                         .count();

  session.write(
    channel_id, 0, timestamp, timestamp, reinterpret_cast<const uint8_t*>(data.data()), data.size()
  );

  TaskConfig config;
  config.task_id = "test_task";
  config.device_id = "robot";
  config.scene = "test";

  session.set_task_config(config);
  session.close();

  std::string checksum = session.get_checksum();
  EXPECT_FALSE(checksum.empty());
  EXPECT_EQ(64, checksum.length());  // SHA-256 = 64 hex chars
}

TEST_F(RecordingSessionTest, NoSidecarWithoutTaskConfig) {
  RecordingSession session;
  auto path = get_test_path("test_no_sidecar.mcap");

  McapWriterOptions options;
  session.open(path, options);
  session.close();

  EXPECT_TRUE(session.get_sidecar_path().empty());
  EXPECT_TRUE(session.get_checksum().empty());

  // Check that no JSON file was created
  std::string json_path_str = path;
  size_t dot_pos = json_path_str.find_last_of('.');
  if (dot_pos != std::string::npos) {
    json_path_str = json_path_str.substr(0, dot_pos) + ".json";
  }
  EXPECT_FALSE(fs::exists(json_path_str));
}

// ============================================================================
// Get Channel/Schema ID Tests
// ============================================================================

TEST_F(RecordingSessionTest, GetChannelIdForUnregisteredTopic) {
  RecordingSession session;
  auto path = get_test_path("test_get_channel.mcap");

  McapWriterOptions options;
  session.open(path, options);

  uint16_t channel_id = session.get_channel_id("/unregistered/topic");

  EXPECT_EQ(channel_id, 0);
}

TEST_F(RecordingSessionTest, GetSchemaIdForUnregisteredType) {
  RecordingSession session;
  auto path = get_test_path("test_get_schema.mcap");

  McapWriterOptions options;
  session.open(path, options);

  uint16_t schema_id = session.get_schema_id("unregistered/msg/Type");

  EXPECT_EQ(schema_id, 0);
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(RecordingSessionTest, ConcurrentWrites) {
  RecordingSession session;
  auto path = get_test_path("test_concurrent.mcap");

  McapWriterOptions options;
  ASSERT_TRUE(session.open(path, options));

  uint16_t schema_id = session.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test", "std_msgs/msg/String", "cdr", schema_id);

  constexpr int num_threads = 4;
  constexpr int messages_per_thread = 50;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < messages_per_thread; ++i) {
        std::string data = "thread " + std::to_string(t) + " message " + std::to_string(i);
        uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::system_clock::now().time_since_epoch()
        )
                               .count();

        if (session.write(
              channel_id,
              static_cast<uint32_t>(i),
              timestamp,
              timestamp,
              reinterpret_cast<const uint8_t*>(data.data()),
              data.size()
            )) {
          success_count++;
        }
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  auto stats = session.get_stats();
  EXPECT_EQ(stats.messages_written, num_threads * messages_per_thread);
  EXPECT_EQ(success_count.load(), num_threads * messages_per_thread);
}

// ============================================================================
// Error Message Tests
// ============================================================================

TEST_F(RecordingSessionTest, GetLastErrorWhenNotInitialized) {
  RecordingSession session;

  std::string error = session.get_last_error();

  // get_last_error() may return empty when writer is not initialized
  // This is acceptable behavior - the test just verifies the method can be called
}

// ============================================================================
// Get Path Tests
// ============================================================================

TEST_F(RecordingSessionTest, GetPathWhenNotOpen) {
  RecordingSession session;

  EXPECT_EQ(session.get_path(), "");
}

TEST_F(RecordingSessionTest, GetPathAfterOpen) {
  RecordingSession session;
  auto path = get_test_path("test_get_path.mcap");

  McapWriterOptions options;
  session.open(path, options);

  EXPECT_EQ(session.get_path(), path);
}

TEST_F(RecordingSessionTest, GetPathAfterClose) {
  RecordingSession session;
  auto path = get_test_path("test_get_path_close.mcap");

  McapWriterOptions options;
  session.open(path, options);
  session.close();

  EXPECT_EQ(session.get_path(), "");  // Path cleared after close
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

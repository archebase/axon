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

}  // namespace
}  // namespace recorder
}  // namespace axon

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


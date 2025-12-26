/**
 * Integration tests for metadata injection end-to-end flow
 *
 * Tests the complete workflow:
 * 1. Create RecordingSession with TaskConfig
 * 2. Record some messages
 * 3. Close session (triggers metadata injection + sidecar generation)
 * 4. Verify MCAP metadata records
 * 5. Verify JSON sidecar content
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>

#include <nlohmann/json.hpp>

#include "mcap_writer_wrapper.hpp"
#include "recording_session.hpp"
#include "task_config.hpp"

using namespace axon::recorder;
using namespace axon::mcap_wrapper;

class MetadataInjectionIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a temp directory for test files
    test_dir_ = std::filesystem::temp_directory_path() / "metadata_injection_integration";
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  TaskConfig create_sample_config() {
    TaskConfig config;
    config.task_id = "task_integration_test_001";
    config.device_id = "test_robot";
    config.data_collector_id = "collector_test";
    config.order_id = "order_test_001";
    config.operator_name = "test.operator";
    config.scene = "integration_test_scene";
    config.subscene = "test_subscene";
    config.skills = {"skill_a", "skill_b"};
    config.factory = "test_factory";
    config.topics = {"/test/topic1", "/test/topic2"};
    return config;
  }

  // Helper to create sample message data
  std::vector<uint8_t> create_sample_message(size_t size = 100) {
    std::vector<uint8_t> data(size);
    for (size_t i = 0; i < size; ++i) {
      data[i] = static_cast<uint8_t>(i % 256);
    }
    return data;
  }

  std::filesystem::path test_dir_;
};

TEST_F(MetadataInjectionIntegrationTest, EndToEndRecordingWithMetadata) {
  std::string mcap_path = (test_dir_ / "integration_test.mcap").string();

  // Create and configure recording session
  RecordingSession session;

  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(session.open(mcap_path, options));

  // Set task config for metadata injection
  auto config = create_sample_config();
  session.set_task_config(config);

  // Register schema and channel
  uint16_t schema_id = session.register_schema("test_msgs/msg/TestMessage", "ros2msg", "uint64 timestamp\nstring data");
  ASSERT_NE(schema_id, 0);

  uint16_t channel_id = session.register_channel("/test/topic1", "cdr", schema_id);
  ASSERT_NE(channel_id, 0);

  // Write some messages
  auto msg_data = create_sample_message(50);
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()
  ).count();

  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(session.write(channel_id, i, now_ns + i * 1000000, now_ns + i * 1000000,
                              msg_data.data(), msg_data.size()));
    session.update_topic_stats("/test/topic1", "test_msgs/msg/TestMessage");
  }

  // Close session (triggers metadata injection)
  session.close();

  // Verify MCAP file exists
  EXPECT_TRUE(std::filesystem::exists(mcap_path));

  // Verify sidecar was generated
  std::string sidecar_path = session.get_sidecar_path();
  EXPECT_FALSE(sidecar_path.empty());
  EXPECT_TRUE(std::filesystem::exists(sidecar_path));

  // Verify checksum was computed
  std::string checksum = session.get_checksum();
  EXPECT_FALSE(checksum.empty());
  EXPECT_EQ(checksum.length(), 64);
}

TEST_F(MetadataInjectionIntegrationTest, SidecarJsonSchema) {
  std::string mcap_path = (test_dir_ / "schema_test.mcap").string();

  RecordingSession session;
  ASSERT_TRUE(session.open(mcap_path));

  auto config = create_sample_config();
  session.set_task_config(config);

  // Register and write a message
  uint16_t schema_id = session.register_schema("test/Message", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test/topic", "cdr", schema_id);
  auto msg = create_sample_message(25);
  auto ts = std::chrono::system_clock::now().time_since_epoch().count();
  EXPECT_TRUE(session.write(channel_id, 0, ts, ts, msg.data(), msg.size()));
  session.update_topic_stats("/test/topic", "test/Message");

  session.close();

  // Parse sidecar JSON
  std::ifstream ifs(session.get_sidecar_path());
  nlohmann::json sidecar;
  ASSERT_NO_THROW(sidecar = nlohmann::json::parse(ifs));

  // Verify schema structure
  EXPECT_EQ(sidecar["version"], "1.0");
  EXPECT_TRUE(sidecar.contains("mcap_file"));
  EXPECT_TRUE(sidecar.contains("task"));
  EXPECT_TRUE(sidecar.contains("device"));
  EXPECT_TRUE(sidecar.contains("recording"));
  EXPECT_TRUE(sidecar.contains("topics_summary"));

  // Verify task metadata
  EXPECT_EQ(sidecar["task"]["task_id"], "task_integration_test_001");
  EXPECT_EQ(sidecar["task"]["scene"], "integration_test_scene");
  EXPECT_EQ(sidecar["task"]["factory"], "test_factory");
  EXPECT_EQ(sidecar["task"]["order_id"], "order_test_001");
  EXPECT_EQ(sidecar["task"]["operator_name"], "test.operator");

  // Verify device metadata
  EXPECT_EQ(sidecar["device"]["device_id"], "test_robot");

  // Verify recording metadata
  EXPECT_TRUE(sidecar["recording"].contains("recorder_version"));
  EXPECT_TRUE(sidecar["recording"].contains("recording_started_at"));
  EXPECT_TRUE(sidecar["recording"].contains("recording_finished_at"));
  EXPECT_TRUE(sidecar["recording"].contains("duration_sec"));
  EXPECT_TRUE(sidecar["recording"].contains("message_count"));
  EXPECT_TRUE(sidecar["recording"].contains("file_size_bytes"));
  EXPECT_TRUE(sidecar["recording"].contains("checksum_sha256"));

  // Verify checksum format
  std::string checksum = sidecar["recording"]["checksum_sha256"];
  EXPECT_EQ(checksum.length(), 64);

  // Verify topics summary
  EXPECT_EQ(sidecar["topics_summary"].size(), 1);
  EXPECT_EQ(sidecar["topics_summary"][0]["topic"], "/test/topic");
  EXPECT_EQ(sidecar["topics_summary"][0]["message_type"], "test/Message");
}

TEST_F(MetadataInjectionIntegrationTest, MultiTopicRecording) {
  std::string mcap_path = (test_dir_ / "multi_topic.mcap").string();

  RecordingSession session;
  ASSERT_TRUE(session.open(mcap_path));
  session.set_task_config(create_sample_config());

  // Register multiple topics
  uint16_t schema1 = session.register_schema("sensor_msgs/Image", "ros2msg", "uint32 height");
  uint16_t schema2 = session.register_schema("sensor_msgs/JointState", "ros2msg", "float64[] position");

  uint16_t channel1 = session.register_channel("/camera/image", "cdr", schema1);
  uint16_t channel2 = session.register_channel("/joint_states", "cdr", schema2);

  auto msg = create_sample_message(100);
  auto ts = std::chrono::system_clock::now().time_since_epoch().count();

  // Write messages to different topics
  for (int i = 0; i < 30; ++i) {
    EXPECT_TRUE(session.write(channel1, i, ts + i * 33000000, ts + i * 33000000, msg.data(), msg.size()));
    session.update_topic_stats("/camera/image", "sensor_msgs/Image");
  }

  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(session.write(channel2, i, ts + i * 10000000, ts + i * 10000000, msg.data(), msg.size()));
    session.update_topic_stats("/joint_states", "sensor_msgs/JointState");
  }

  session.close();

  // Parse sidecar
  std::ifstream ifs(session.get_sidecar_path());
  nlohmann::json sidecar;
  ASSERT_NO_THROW(sidecar = nlohmann::json::parse(ifs));

  // Verify topics summary has both topics
  EXPECT_EQ(sidecar["topics_summary"].size(), 2);

  // Verify message counts
  bool found_camera = false, found_joints = false;
  for (const auto& topic : sidecar["topics_summary"]) {
    if (topic["topic"] == "/camera/image") {
      EXPECT_EQ(topic["message_count"], 30);
      found_camera = true;
    } else if (topic["topic"] == "/joint_states") {
      EXPECT_EQ(topic["message_count"], 100);
      found_joints = true;
    }
  }
  EXPECT_TRUE(found_camera);
  EXPECT_TRUE(found_joints);
}

TEST_F(MetadataInjectionIntegrationTest, EmptyRecordingStillHasMetadata) {
  std::string mcap_path = (test_dir_ / "empty.mcap").string();

  RecordingSession session;
  ASSERT_TRUE(session.open(mcap_path));
  session.set_task_config(create_sample_config());

  // Close without writing any messages
  session.close();

  // Sidecar should still be generated
  EXPECT_TRUE(std::filesystem::exists(session.get_sidecar_path()));

  // Parse and verify
  std::ifstream ifs(session.get_sidecar_path());
  nlohmann::json sidecar;
  ASSERT_NO_THROW(sidecar = nlohmann::json::parse(ifs));

  EXPECT_EQ(sidecar["recording"]["message_count"], 0);
  EXPECT_EQ(sidecar["topics_summary"].size(), 0);
}

TEST_F(MetadataInjectionIntegrationTest, WithoutTaskConfigNoSidecar) {
  std::string mcap_path = (test_dir_ / "no_config.mcap").string();

  RecordingSession session;
  ASSERT_TRUE(session.open(mcap_path));

  // Don't set task config

  uint16_t schema_id = session.register_schema("test/Message", "ros2msg", "string data");
  uint16_t channel_id = session.register_channel("/test/topic", "cdr", schema_id);
  auto msg = create_sample_message(25);
  auto ts = std::chrono::system_clock::now().time_since_epoch().count();
  EXPECT_TRUE(session.write(channel_id, 0, ts, ts, msg.data(), msg.size()));

  session.close();

  // MCAP should exist
  EXPECT_TRUE(std::filesystem::exists(mcap_path));

  // But no sidecar
  EXPECT_TRUE(session.get_sidecar_path().empty());
}

TEST_F(MetadataInjectionIntegrationTest, ISO8601TimestampFormat) {
  std::string mcap_path = (test_dir_ / "timestamp.mcap").string();

  RecordingSession session;
  ASSERT_TRUE(session.open(mcap_path));
  session.set_task_config(create_sample_config());
  session.close();

  std::ifstream ifs(session.get_sidecar_path());
  nlohmann::json sidecar;
  ASSERT_NO_THROW(sidecar = nlohmann::json::parse(ifs));

  // Check ISO8601 format (YYYY-MM-DDTHH:MM:SS.mmmZ)
  std::string started = sidecar["recording"]["recording_started_at"];
  std::string finished = sidecar["recording"]["recording_finished_at"];

  // Basic format check
  EXPECT_EQ(started[4], '-');
  EXPECT_EQ(started[7], '-');
  EXPECT_EQ(started[10], 'T');
  EXPECT_EQ(started[13], ':');
  EXPECT_EQ(started[16], ':');
  EXPECT_EQ(started[19], '.');
  EXPECT_EQ(started.back(), 'Z');
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


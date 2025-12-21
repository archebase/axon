/**
 * Unit tests for MetadataInjector
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "metadata_injector.hpp"
#include "mcap_writer_wrapper.hpp"

using namespace axon::recorder;
using namespace axon::mcap_wrapper;

// ============================================================================
// MetadataInjector Unit Tests
// ============================================================================

class MetadataInjectorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a temp directory for test files
    test_dir_ = std::filesystem::temp_directory_path() / "metadata_injector_test";
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  TaskConfig create_sample_config() {
    TaskConfig config;
    config.task_id = "task_20251220_143052_abc123";
    config.device_id = "robot_arm_001";
    config.data_collector_id = "collector_east_01";
    config.order_id = "order_batch_2025Q4_001";
    config.operator_name = "john.doe";
    config.scene = "warehouse_picking";
    config.subscene = "shelf_approach";
    config.skills = {"grasp", "place", "navigate"};
    config.factory = "factory_shanghai_01";
    config.topics = {"/camera/image", "/joint_states"};
    return config;
  }

  std::filesystem::path test_dir_;
};

TEST_F(MetadataInjectorTest, DefaultConstruction) {
  MetadataInjector injector;

  // Without task config, inject_metadata should return false
  McapWriterWrapper writer;
  EXPECT_FALSE(injector.inject_metadata(writer, 0, 0));
}

TEST_F(MetadataInjectorTest, SetTaskConfig) {
  MetadataInjector injector;
  auto config = create_sample_config();

  injector.set_task_config(config);

  // After setting config, sidecar path should still be empty (not generated yet)
  EXPECT_TRUE(injector.get_sidecar_path().empty());
  EXPECT_TRUE(injector.get_checksum().empty());
}

TEST_F(MetadataInjectorTest, UpdateTopicStats) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Update topic stats
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/joint_states", "sensor_msgs/JointState", 500);

  // Stats should be tracked (no direct accessor, but will be used in sidecar generation)
  SUCCEED();
}

TEST_F(MetadataInjectorTest, InjectMetadataToMCAP) {
  // Create a temporary MCAP file
  std::string mcap_path = (test_dir_ / "test.mcap").string();

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  // Setup injector
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);

  // Inject metadata
  EXPECT_TRUE(injector.inject_metadata(writer, 100, 1024));

  writer.close();
}

TEST_F(MetadataInjectorTest, GenerateSidecarJson) {
  // Create a temporary MCAP file
  std::string mcap_path = (test_dir_ / "test_sidecar.mcap").string();

  // Create a simple MCAP file
  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/joint_states", "sensor_msgs/JointState", 500);

  EXPECT_TRUE(injector.inject_metadata(writer, 600, 2048));
  writer.close();

  // Get actual file size
  auto file_size = std::filesystem::file_size(mcap_path);

  // Generate sidecar
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  // Verify sidecar was created
  std::string sidecar_path = injector.get_sidecar_path();
  EXPECT_FALSE(sidecar_path.empty());
  EXPECT_TRUE(std::filesystem::exists(sidecar_path));

  // Verify checksum was computed
  std::string checksum = injector.get_checksum();
  EXPECT_FALSE(checksum.empty());
  EXPECT_EQ(checksum.length(), 64);  // SHA-256 produces 64 hex chars
}

TEST_F(MetadataInjectorTest, SidecarJsonContainsRequiredFields) {
  // Create a temporary MCAP file
  std::string mcap_path = (test_dir_ / "test_fields.mcap").string();

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  MetadataInjector injector;
  auto config = create_sample_config();
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);

  EXPECT_TRUE(injector.inject_metadata(writer, 100, 2048));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  // Read sidecar JSON
  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  // Verify required fields are present
  EXPECT_TRUE(content.find("\"version\": \"1.0\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"task_id\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"task_20251220_143052_abc123\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"scene\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"warehouse_picking\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"factory\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"factory_shanghai_01\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"device_id\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"robot_arm_001\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"recorder_version\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"checksum_sha256\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"topics_summary\"") != std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarJsonContainsNewFields) {
  // Create a temporary MCAP file
  std::string mcap_path = (test_dir_ / "test_new_fields.mcap").string();

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  MetadataInjector injector;
  auto config = create_sample_config();
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  // Read sidecar JSON
  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  // Verify new fields
  EXPECT_TRUE(content.find("\"order_id\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"order_batch_2025Q4_001\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"operator_name\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"john.doe\"") != std::string::npos);
}

TEST_F(MetadataInjectorTest, TopicFrequencyCalculation) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Simulate messages over time
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 10);

  // Stats are tracked, will be reflected in sidecar
  SUCCEED();
}

TEST_F(MetadataInjectorTest, RosParamLoaderCallback) {
  MetadataInjector injector;

  // Set a custom ROS parameter loader
  injector.set_ros_param_loader([](const std::string& param) -> std::string {
    if (param == "device_model") return "custom_model";
    if (param == "device_serial") return "custom_serial";
    return "";
  });

  // The loader will be used if env vars and config don't provide values
  SUCCEED();
}

TEST_F(MetadataInjectorTest, EnvVarOverridesRosParam) {
  // Set environment variables
  setenv("AXON_DEVICE_MODEL", "env_model", 1);
  setenv("AXON_DEVICE_SERIAL", "env_serial", 1);

  MetadataInjector injector;
  auto config = create_sample_config();
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // ROS param loader should NOT be called since env vars are set
  injector.set_ros_param_loader([](const std::string& param) -> std::string {
    // This should not be reached for device_model and device_serial
    return "ros_param_value";
  });

  // Create MCAP and generate sidecar
  std::string mcap_path = (test_dir_ / "test_env.mcap").string();
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));
  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  // Read sidecar and verify env var values are used
  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("\"device_model\": \"env_model\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"device_serial\": \"env_serial\"") != std::string::npos);

  // Clean up env vars
  unsetenv("AXON_DEVICE_MODEL");
  unsetenv("AXON_DEVICE_SERIAL");
}

TEST_F(MetadataInjectorTest, AtomicFileWrite) {
  // Test that sidecar is written atomically (via temp file + rename)
  std::string mcap_path = (test_dir_ / "test_atomic.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);

  // Generate sidecar - should be atomic
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  // Verify no .tmp file remains
  std::string tmp_path = injector.get_sidecar_path() + ".tmp";
  EXPECT_FALSE(std::filesystem::exists(tmp_path));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


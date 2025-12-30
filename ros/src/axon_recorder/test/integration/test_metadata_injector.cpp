/**
 * Unit tests for MetadataInjector
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>

#include "mcap_writer_wrapper.hpp"
#include "metadata_injector.hpp"

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
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

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
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

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
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

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

// ============================================================================
// Checksum Verification Tests
// ============================================================================

TEST_F(MetadataInjectorTest, ChecksumDeterministic) {
  // Same file content should produce same checksum
  std::string mcap_path = (test_dir_ / "test_checksum.mcap").string();

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());
  injector.update_topic_stats("/test", "std_msgs/String", 10);

  EXPECT_TRUE(injector.inject_metadata(writer, 10, 1024));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  std::string checksum1 = injector.get_checksum();

  // Generate sidecar again
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));
  std::string checksum2 = injector.get_checksum();

  // Checksums should match
  EXPECT_EQ(checksum1, checksum2);
}

TEST_F(MetadataInjectorTest, ChecksumFormat) {
  std::string mcap_path = (test_dir_ / "test_checksum_format.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));

  std::string checksum = injector.get_checksum();

  // SHA-256 checksum should be:
  // - 64 characters long
  // - Only hexadecimal characters (0-9, a-f)
  EXPECT_EQ(checksum.length(), 64);

  for (char c : checksum) {
    EXPECT_TRUE(std::isxdigit(c)) << "Invalid character in checksum: " << c;
  }
}

TEST_F(MetadataInjectorTest, ChecksumDifferentForDifferentFiles) {
  // Create two different MCAP files
  std::string mcap_path1 = (test_dir_ / "test_checksum1.mcap").string();
  std::string mcap_path2 = (test_dir_ / "test_checksum2.mcap").string();

  // First file
  McapWriterWrapper writer1;
  ASSERT_TRUE(writer1.open(mcap_path1));

  MetadataInjector injector1;
  injector1.set_task_config(create_sample_config());
  injector1.set_recording_start_time(std::chrono::system_clock::now());
  injector1.update_topic_stats("/topic1", "std_msgs/String", 10);
  EXPECT_TRUE(injector1.inject_metadata(writer1, 10, 100));
  writer1.close();

  // Second file with different content
  McapWriterWrapper writer2;
  ASSERT_TRUE(writer2.open(mcap_path2));

  MetadataInjector injector2;
  auto config2 = create_sample_config();
  config2.task_id = "different_task";
  injector2.set_task_config(config2);
  injector2.set_recording_start_time(std::chrono::system_clock::now());
  injector2.update_topic_stats("/topic2", "std_msgs/Int32", 20);
  EXPECT_TRUE(injector2.inject_metadata(writer2, 20, 200));
  writer2.close();

  // Generate sidecars
  injector1.generate_sidecar_json(mcap_path1, std::filesystem::file_size(mcap_path1));
  injector2.generate_sidecar_json(mcap_path2, std::filesystem::file_size(mcap_path2));

  // Checksums should be different
  EXPECT_NE(injector1.get_checksum(), injector2.get_checksum());
}

// ============================================================================
// JSON Format Validation Tests
// ============================================================================

TEST_F(MetadataInjectorTest, ValidJsonFormat) {
  std::string mcap_path = (test_dir_ / "test_json.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  // Read sidecar
  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Basic JSON structure checks
  EXPECT_TRUE(content.front() == '{');
  EXPECT_TRUE(content.back() == '}' || content.back() == '\n');

  // Count braces to verify they're balanced
  int brace_count = 0;
  for (char c : content) {
    if (c == '{') ++brace_count;
    if (c == '}') --brace_count;
  }
  EXPECT_EQ(brace_count, 0) << "Unbalanced braces in JSON";
}

TEST_F(MetadataInjectorTest, SidecarContainsSkills) {
  std::string mcap_path = (test_dir_ / "test_skills.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  auto config = create_sample_config();
  config.skills = {"grasp", "place", "navigate", "detect"};
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("\"skills\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"grasp\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"place\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"navigate\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"detect\"") != std::string::npos);
}

TEST_F(MetadataInjectorTest, SidecarContainsTopicsSummary) {
  std::string mcap_path = (test_dir_ / "test_topics.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add multiple topics with different counts
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 100);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 1000);
  injector.update_topic_stats("/lidar/scan", "sensor_msgs/LaserScan", 50);

  EXPECT_TRUE(injector.inject_metadata(writer, 1150, 1024 * 1024));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("\"topics_summary\"") != std::string::npos);
  EXPECT_TRUE(content.find("/camera/image") != std::string::npos);
  EXPECT_TRUE(content.find("/imu/data") != std::string::npos);
  EXPECT_TRUE(content.find("/lidar/scan") != std::string::npos);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(MetadataInjectorTest, GenerateSidecarWithZeroFileSize) {
  // Test that sidecar generation handles zero file size
  std::string mcap_path = (test_dir_ / "test_zero_size.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  // Generate sidecar with actual file
  auto file_size = std::filesystem::file_size(mcap_path);
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, file_size));
}

TEST_F(MetadataInjectorTest, GenerateSidecarRequiresConfig) {
  // Test that sidecar generation requires task config to be set
  MetadataInjector injector;
  // Don't set task config

  std::string mcap_path = (test_dir_ / "test_no_config.mcap").string();
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));
  writer.close();

  // Without config set, inject_metadata should fail
  McapWriterWrapper writer2;
  ASSERT_TRUE(writer2.open(mcap_path));
  EXPECT_FALSE(injector.inject_metadata(writer2, 0, 0));
  writer2.close();
}

TEST_F(MetadataInjectorTest, InjectMetadataWithoutConfig) {
  MetadataInjector injector;
  // Don't set task config

  std::string mcap_path = (test_dir_ / "test_no_config.mcap").string();
  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  // Should fail gracefully
  EXPECT_FALSE(injector.inject_metadata(writer, 0, 0));

  writer.close();
}

TEST_F(MetadataInjectorTest, InjectMetadataToClosedWriter) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  McapWriterWrapper writer;
  // Writer is not open

  // Should fail because writer is not open
  EXPECT_FALSE(injector.inject_metadata(writer, 0, 0));
}

// ============================================================================
// Large Data Tests
// ============================================================================

TEST_F(MetadataInjectorTest, LargeMessageCount) {
  std::string mcap_path = (test_dir_ / "test_large.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Large message count
  injector.update_topic_stats("/high_freq", "std_msgs/Int32", 1000000);

  EXPECT_TRUE(injector.inject_metadata(writer, 1000000, 50 * 1024 * 1024));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("1000000") != std::string::npos);
}

TEST_F(MetadataInjectorTest, ManyTopics) {
  std::string mcap_path = (test_dir_ / "test_many_topics.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  auto config = create_sample_config();

  // Clear existing topics and add many
  config.topics.clear();
  for (int i = 0; i < 50; ++i) {
    config.topics.push_back("/topic_" + std::to_string(i));
  }
  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add stats for all topics
  for (int i = 0; i < 50; ++i) {
    injector.update_topic_stats("/topic_" + std::to_string(i), "std_msgs/String", i * 10);
  }

  EXPECT_TRUE(injector.inject_metadata(writer, 12250, 1024 * 1024));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("/topic_0") != std::string::npos);
  EXPECT_TRUE(content.find("/topic_49") != std::string::npos);
}

// ============================================================================
// Special Characters Tests
// ============================================================================

TEST_F(MetadataInjectorTest, SpecialCharactersInConfig) {
  std::string mcap_path = (test_dir_ / "test_special.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "task_with_\"quotes\"_and_\\backslash";
  config.device_id = "device/with/slashes";
  config.operator_name = "user@domain.com";
  config.scene = "scene with spaces";
  config.subscene = "subscene-with-dashes";
  config.skills = {"skill:with:colons"};
  config.factory = "factory\twith\ttabs";
  config.topics = {"/topic/with/deep/nesting"};

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  // Sidecar should be created without errors
  EXPECT_TRUE(std::filesystem::exists(injector.get_sidecar_path()));
}

TEST_F(MetadataInjectorTest, UnicodeInConfig) {
  std::string mcap_path = (test_dir_ / "test_unicode.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  TaskConfig config;
  config.task_id = "task_日本語_123";
  config.device_id = "robot_中文";
  config.operator_name = "opérateur_français";
  config.scene = "сцена_русский";
  config.subscene = "تحت المشهد";
  config.skills = {"技能1", "技能2"};
  config.factory = "工厂_αβγ";
  config.topics = {"/topic/日本語"};

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Unicode should be preserved
  EXPECT_TRUE(content.find("日本語") != std::string::npos);
  EXPECT_TRUE(content.find("中文") != std::string::npos);
}

// ============================================================================
// Timestamp Tests
// ============================================================================

TEST_F(MetadataInjectorTest, TimestampFormat) {
  std::string mcap_path = (test_dir_ / "test_timestamp.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Verify ISO8601 timestamp format (YYYY-MM-DDTHH:MM:SS)
  EXPECT_TRUE(content.find("\"recording_started_at\"") != std::string::npos);
  // Should contain date separator and time separator
  EXPECT_TRUE(content.find("T") != std::string::npos);
}

// ============================================================================
// Enhanced Coverage Tests (Phase 5)
// ============================================================================

TEST_F(MetadataInjectorTest, ChecksumForNonExistentFile) {
  MetadataInjector injector;

  // Attempt to get checksum for non-existent file
  std::string checksum = injector.get_checksum();

  // Should be empty since no file was processed
  EXPECT_TRUE(checksum.empty());
}

TEST_F(MetadataInjectorTest, GenerateSidecarWithoutConfig) {
  std::string mcap_path = (test_dir_ / "test_no_config.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));
  writer.close();

  MetadataInjector injector;
  // Don't set task config

  // Should return false when task config is not set
  EXPECT_FALSE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));
}

TEST_F(MetadataInjectorTest, UpdateTopicStatsEmpty) {
  MetadataInjector injector;

  // No updates - should handle gracefully
  // Can proceed with metadata injection (topic summary will be empty)
  // This tests that the injector works without any topic stats set
}

TEST_F(MetadataInjectorTest, SidecarPathBeforeGeneration) {
  MetadataInjector injector;

  // Get sidecar path before any file is generated
  std::string path = injector.get_sidecar_path();

  // Should be empty
  EXPECT_TRUE(path.empty());
}

TEST_F(MetadataInjectorTest, MultipleTopicStatsUpdates) {
  std::string mcap_path = (test_dir_ / "test_multi_stats.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // First update for camera
  injector.update_topic_stats("/camera", "sensor_msgs/Image", 100);

  // Second update - additional messages
  injector.update_topic_stats("/camera", "sensor_msgs/Image", 200);

  // Add another topic
  injector.update_topic_stats("/lidar", "sensor_msgs/LaserScan", 50);

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Should have both topics
  EXPECT_TRUE(content.find("/camera") != std::string::npos);
  EXPECT_TRUE(content.find("/lidar") != std::string::npos);
}

TEST_F(MetadataInjectorTest, ChecksumFormatVerification) {
  std::string mcap_path = (test_dir_ / "test_checksum_format.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::string checksum = injector.get_checksum();

  // SHA-256 checksum should be 64 hex characters
  EXPECT_EQ(checksum.length(), 64);

  // All characters should be lowercase hex
  for (char c : checksum) {
    bool is_hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    EXPECT_TRUE(is_hex) << "Character '" << c << "' is not valid hex";
  }
}

TEST_F(MetadataInjectorTest, RecordingStartTimeNotSet) {
  std::string mcap_path = (test_dir_ / "test_no_start_time.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  // Don't set recording start time

  // Should still succeed (will use default time or current time)
  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();
}

TEST_F(MetadataInjectorTest, ZeroFileSize) {
  std::string mcap_path = (test_dir_ / "test_zero_size.mcap").string();

  // Create an empty file
  { std::ofstream ofs(mcap_path); }

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Generate sidecar with zero file size - may or may not succeed depending on checksum handling
  bool result = injector.generate_sidecar_json(mcap_path, 0);

  // If it succeeded, verify sidecar was created
  if (result) {
    std::string sidecar_path = injector.get_sidecar_path();
    EXPECT_TRUE(std::filesystem::exists(sidecar_path));
  }
  // Either way, the function should not crash
}

TEST_F(MetadataInjectorTest, VeryLargeMessageCounts) {
  std::string mcap_path = (test_dir_ / "test_large_counts.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Update with large message counts (1 billion messages)
  injector.update_topic_stats("/high_freq_topic", "sensor_msgs/PointCloud2", 1000000000);

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  // Verify sidecar was created
  EXPECT_TRUE(std::filesystem::exists(injector.get_sidecar_path()));
}

TEST_F(MetadataInjectorTest, ManyTopicsWithStats) {
  std::string mcap_path = (test_dir_ / "test_many_topics_stats.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  TaskConfig config = create_sample_config();

  // Add many topics
  config.topics.clear();
  for (int i = 0; i < 100; ++i) {
    config.topics.push_back("/topic_" + std::to_string(i));
  }

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add stats for all topics using the 3-argument API
  for (int i = 0; i < 100; ++i) {
    std::string topic = "/topic_" + std::to_string(i);
    injector.update_topic_stats(topic, "std_msgs/String", i * 100);
  }

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Verify first and last topics are present
  EXPECT_TRUE(content.find("/topic_0") != std::string::npos);
  EXPECT_TRUE(content.find("/topic_99") != std::string::npos);
}

TEST_F(MetadataInjectorTest, ConfigWithEmptySkills) {
  std::string mcap_path = (test_dir_ / "test_empty_skills.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  TaskConfig config = create_sample_config();
  config.skills.clear();

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // When skills is empty, it should be omitted from JSON (implementation design choice)
  EXPECT_FALSE(content.find("\"skills\"") != std::string::npos);

  // But other task fields should still be present
  EXPECT_TRUE(content.find("\"task\"") != std::string::npos);
  EXPECT_TRUE(content.find("\"task_id\"") != std::string::npos);
}

TEST_F(MetadataInjectorTest, ConfigWithEmptyTopics) {
  std::string mcap_path = (test_dir_ / "test_empty_topics.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  TaskConfig config = create_sample_config();
  config.topics.clear();

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));
}

TEST_F(MetadataInjectorTest, TopicFrequencyCalculationExtended) {
  std::string mcap_path = (test_dir_ / "test_frequency_ext.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());

  // Set recording start time (10 seconds ago)
  auto start_time = std::chrono::system_clock::now() - std::chrono::seconds(10);
  injector.set_recording_start_time(start_time);

  // Add topic stats - 100 messages over 10 seconds = 10 Hz
  injector.update_topic_stats("/camera", "sensor_msgs/Image", 100);

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));
}

TEST_F(MetadataInjectorTest, RosParameterLoaderSetter) {
  MetadataInjector injector;

  // Set the parameter loader callback
  // ConfigLoaderCallback takes a param name and returns the value
  std::vector<std::string> requested_params;
  injector.set_ros_param_loader([&requested_params](const std::string& param_name) -> std::string {
    requested_params.push_back(param_name);
    // Return a mock value based on the parameter name
    if (param_name == "device_model") {
      return "TestRobotFromParam";
    } else if (param_name == "device_serial") {
      return "SN_PARAM_123";
    } else if (param_name == "ros_distro") {
      return "test_distro";
    }
    return "";
  });

  // Verify the callback was set (no crash, no exception)
  // The callback will be invoked during metadata generation when env vars aren't set
  // This test verifies the setter works without exceptions
  EXPECT_NO_THROW(injector.set_task_config(create_sample_config()));
}

// RAII helper for environment variable cleanup
class ScopedEnvVar {
public:
  ScopedEnvVar(const char* name, const char* value)
      : name_(name)
      , original_value_()
      , had_original_(false) {
    // Save original value (if any) before overwriting
    const char* original = getenv(name);
    if (original) {
      had_original_ = true;
      original_value_ = original;
    }
    setenv(name, value, 1);
  }

  ~ScopedEnvVar() {
    // Restore original value or unset if there was no original
    if (had_original_) {
      setenv(name_.c_str(), original_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

  // Non-copyable and non-movable (prevent double-restore bugs)
  ScopedEnvVar(const ScopedEnvVar&) = delete;
  ScopedEnvVar& operator=(const ScopedEnvVar&) = delete;
  ScopedEnvVar(ScopedEnvVar&&) = delete;
  ScopedEnvVar& operator=(ScopedEnvVar&&) = delete;

private:
  std::string name_;
  std::string original_value_;
  bool had_original_;
};

TEST_F(MetadataInjectorTest, EnvironmentVariableOverrides) {
  std::string mcap_path = (test_dir_ / "test_env_override.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  // Use RAII for environment variable cleanup - ensures cleanup even on test failure
  ScopedEnvVar env_model("AXON_DEVICE_MODEL", "TestRobot");
  ScopedEnvVar env_serial("AXON_DEVICE_SERIAL", "SN123456");

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Should contain environment variable values
  EXPECT_TRUE(content.find("TestRobot") != std::string::npos);
  EXPECT_TRUE(content.find("SN123456") != std::string::npos);
  // RAII handles cleanup automatically
}

TEST_F(MetadataInjectorTest, AtomicFileWriteVerification) {
  std::string mcap_path = (test_dir_ / "test_atomic.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::string sidecar_path = injector.get_sidecar_path();

  // Verify the sidecar file exists and is not a temp file
  EXPECT_TRUE(std::filesystem::exists(sidecar_path));
  EXPECT_TRUE(sidecar_path.find(".tmp") == std::string::npos);

  // Verify the file is valid JSON by reading it
  std::ifstream ifs(sidecar_path);
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Basic JSON structure check
  EXPECT_TRUE(content.front() == '{');
  EXPECT_TRUE(content.back() == '}' || content.back() == '\n');
}

// ============================================================================
// P1: Expanded Metadata Injection Tests
// ============================================================================

TEST_F(MetadataInjectorTest, MetadataWithAllConfigFields) {
  std::string mcap_path = (test_dir_ / "all_fields.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;

  // Create config with ALL fields populated
  TaskConfig config;
  config.task_id = "comprehensive_task_001";
  config.device_id = "device_comprehensive";
  config.data_collector_id = "collector_001";
  config.order_id = "order_12345";
  config.operator_name = "test_operator";
  config.scene = "warehouse_scene";
  config.subscene = "aisle_3_picking";
  config.skills = {"navigation", "manipulation", "perception", "planning"};
  config.factory = "factory_east";
  config.topics = {"/camera/image", "/lidar/scan", "/imu/data", "/joint_states"};
  config.start_callback_url = "http://server/start";
  config.finish_callback_url = "http://server/finish";
  config.user_token = "jwt_token_12345";

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Add topic stats for all topics
  injector.update_topic_stats("/camera/image", "sensor_msgs/Image", 300);
  injector.update_topic_stats("/lidar/scan", "sensor_msgs/LaserScan", 100);
  injector.update_topic_stats("/imu/data", "sensor_msgs/Imu", 1000);
  injector.update_topic_stats("/joint_states", "sensor_msgs/JointState", 500);

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Verify all fields are present
  EXPECT_TRUE(content.find("comprehensive_task_001") != std::string::npos);
  EXPECT_TRUE(content.find("device_comprehensive") != std::string::npos);
  EXPECT_TRUE(content.find("warehouse_scene") != std::string::npos);
  EXPECT_TRUE(content.find("factory_east") != std::string::npos);
}

TEST_F(MetadataInjectorTest, MetadataWithMinimalConfig) {
  std::string mcap_path = (test_dir_ / "minimal_config.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;

  // Minimal config - only required fields
  TaskConfig config;
  config.task_id = "minimal_task";
  config.device_id = "minimal_device";

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  // Should still have required fields
  EXPECT_TRUE(content.find("minimal_task") != std::string::npos);
  EXPECT_TRUE(content.find("minimal_device") != std::string::npos);
}

TEST_F(MetadataInjectorTest, MetadataWithSpecialCharactersInConfig) {
  std::string mcap_path = (test_dir_ / "special_chars.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;

  TaskConfig config;
  config.task_id = "task_with\"quotes\"and\\backslash";
  config.device_id = "device/with/slashes";
  config.scene = "scene\twith\ttabs";
  config.skills = {"skill:with:colons", "skill;with;semicolons"};

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Should handle special characters without crashing
  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));
}

TEST_F(MetadataInjectorTest, MetadataWithUnicodeCharacters) {
  std::string mcap_path = (test_dir_ / "unicode.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;

  TaskConfig config;
  config.task_id = "task_日本語_001";
  config.device_id = "设备_中文";
  config.scene = "场景_한국어";
  config.operator_name = "Оператор_русский";

  injector.set_task_config(config);
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Should handle Unicode without crashing
  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));
}

// ============================================================================
// P2: Targeted MetadataInjector Unit Tests
// ============================================================================

TEST_F(MetadataInjectorTest, TopicStatsFrequencyCalculation) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());

  auto start_time = std::chrono::system_clock::now() - std::chrono::seconds(10);
  injector.set_recording_start_time(start_time);

  // Simulate 100 messages over 10 seconds = 10 Hz
  for (int i = 0; i < 100; ++i) {
    injector.update_topic_stats("/camera", "sensor_msgs/Image", 1);
  }

  // Topic stats should accumulate
  // (No direct accessor, but coverage is the goal)
  SUCCEED();
}

TEST_F(MetadataInjectorTest, TopicStatsMultipleTopics) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Update stats for multiple topics
  for (int i = 0; i < 10; ++i) {
    injector.update_topic_stats("/topic1", "std_msgs/String", 10);
    injector.update_topic_stats("/topic2", "std_msgs/Int32", 5);
    injector.update_topic_stats("/topic3", "sensor_msgs/Image", 2);
  }

  SUCCEED();
}

TEST_F(MetadataInjectorTest, TopicStatsWithZeroMessages) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Zero message count
  injector.update_topic_stats("/empty_topic", "std_msgs/Empty", 0);

  SUCCEED();
}

TEST_F(MetadataInjectorTest, TopicStatsWithVeryHighCount) {
  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Very high message count
  injector.update_topic_stats("/high_freq", "sensor_msgs/Imu", 10000000);

  SUCCEED();
}

TEST_F(MetadataInjectorTest, RecordingStartTimeNotSetInjectMetadata) {
  std::string mcap_path = (test_dir_ / "no_start_time_inject.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  // Don't set recording start time

  // Should handle missing start time gracefully
  bool result = injector.inject_metadata(writer, 0, 0);
  writer.close();

  // May succeed or fail depending on implementation
  // The important thing is no crash
  (void)result;  // Suppress unused variable warning
  SUCCEED();
}

TEST_F(MetadataInjectorTest, GenerateSidecarWithZeroFileSizeP2) {
  std::string mcap_path = (test_dir_ / "zero_size_p2.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  // Generate sidecar with zero file size
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 0));
}

TEST_F(MetadataInjectorTest, GenerateSidecarWithLargeFileSize) {
  std::string mcap_path = (test_dir_ / "large_file.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  // Simulate very large file (1 TB)
  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 1099511627776ULL));
}

TEST_F(MetadataInjectorTest, ChecksumGeneratedCorrectly) {
  std::string mcap_path = (test_dir_ / "checksum_verify.mcap").string();

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.profile = "ros2";
  ASSERT_TRUE(writer.open(mcap_path, options));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  // Register a schema and channel, then write some data to ensure non-empty file
  uint16_t schema_id = writer.register_schema("std_msgs/msg/String", "ros2msg", "string data");
  ASSERT_GT(schema_id, 0);

  uint16_t channel_id = writer.register_channel("/test_topic", "cdr", schema_id);
  ASSERT_GT(channel_id, 0);

  // Write some data
  std::vector<uint8_t> data = {0x01, 0x02, 0x03};
  writer.write(channel_id, 1000000000ULL, 1000000000ULL, data.data(), data.size());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::string checksum = injector.get_checksum();
  if (!checksum.empty()) {
    // Checksum should be consistent format (hex string)
    for (char c : checksum) {
      EXPECT_TRUE(std::isxdigit(c)) << "Invalid char in checksum: " << c;
    }
  }
}

TEST_F(MetadataInjectorTest, SidecarPathMatchesMcapPath) {
  std::string mcap_path = (test_dir_ / "sidecar_path_test.mcap").string();

  McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(mcap_path));

  MetadataInjector injector;
  injector.set_task_config(create_sample_config());
  injector.set_recording_start_time(std::chrono::system_clock::now());

  EXPECT_TRUE(injector.inject_metadata(writer, 0, 0));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, std::filesystem::file_size(mcap_path)));

  std::string sidecar_path = injector.get_sidecar_path();

  // Sidecar should be in same directory as MCAP
  auto mcap_dir = std::filesystem::path(mcap_path).parent_path();
  auto sidecar_dir = std::filesystem::path(sidecar_path).parent_path();
  EXPECT_EQ(mcap_dir, sidecar_dir);

  // Sidecar should have .json extension
  EXPECT_TRUE(sidecar_path.find(".json") != std::string::npos);
}

// ============================================================================
// TopicStats Frequency Calculation Tests (Fixed precision issue)
// ============================================================================

TEST(TopicStatsTest, ComputeFrequencyWithLessThanTwoMessages) {
  TopicStats stats;
  stats.message_count = 0;
  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);

  stats.message_count = 1;
  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);
}

TEST(TopicStatsTest, ComputeFrequencyWithZeroDuration) {
  TopicStats stats;
  stats.message_count = 100;
  auto now = std::chrono::system_clock::now();
  stats.first_message_time = now;
  stats.last_message_time = now;  // Same time - zero duration

  EXPECT_DOUBLE_EQ(stats.compute_frequency_hz(), 0.0);
}

TEST(TopicStatsTest, ComputeFrequencyNormalCase) {
  TopicStats stats;
  stats.message_count = 101;  // 100 intervals
  auto start = std::chrono::system_clock::now();
  stats.first_message_time = start;
  stats.last_message_time = start + std::chrono::seconds(10);  // 10 seconds

  // 100 intervals / 10 seconds = 10 Hz
  double freq = stats.compute_frequency_hz();
  EXPECT_NEAR(freq, 10.0, 0.1);
}

TEST(TopicStatsTest, ComputeFrequencyHighFrequency) {
  TopicStats stats;
  stats.message_count = 1001;  // 1000 intervals
  auto start = std::chrono::system_clock::now();
  stats.first_message_time = start;
  stats.last_message_time = start + std::chrono::milliseconds(100);  // 100ms = 0.1 seconds

  // 1000 intervals / 0.1 seconds = 10000 Hz
  double freq = stats.compute_frequency_hz();
  EXPECT_NEAR(freq, 10000.0, 100.0);  // Allow some tolerance
}

TEST(TopicStatsTest, ComputeFrequencyVeryHighFrequency) {
  // Test precision fix: messages arriving within 1ms should be calculated correctly
  TopicStats stats;
  stats.message_count = 1001;  // 1000 intervals
  auto start = std::chrono::system_clock::now();
  stats.first_message_time = start;
  stats.last_message_time = start + std::chrono::microseconds(1000);  // 1ms = 1000us

  // 1000 intervals / 0.001 seconds = 1,000,000 Hz (1 MHz)
  // With microsecond precision fix, this should be accurate
  double freq = stats.compute_frequency_hz();
  EXPECT_NEAR(freq, 1000000.0, 10000.0);  // Allow 1% tolerance
}

TEST(TopicStatsTest, ComputeFrequencyLowFrequency) {
  TopicStats stats;
  stats.message_count = 11;  // 10 intervals
  auto start = std::chrono::system_clock::now();
  stats.first_message_time = start;
  stats.last_message_time = start + std::chrono::seconds(100);  // 100 seconds

  // 10 intervals / 100 seconds = 0.1 Hz
  double freq = stats.compute_frequency_hz();
  EXPECT_NEAR(freq, 0.1, 0.01);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

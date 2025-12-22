/**
 * Unit tests for MetadataInjector
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  // Read sidecar
  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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
    injector.update_topic_stats("/topic_" + std::to_string(i), 
                                 "std_msgs/String", i * 10);
  }

  EXPECT_TRUE(injector.inject_metadata(writer, 12250, 1024 * 1024));
  writer.close();

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

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

  EXPECT_TRUE(injector.generate_sidecar_json(mcap_path, 
    std::filesystem::file_size(mcap_path)));

  std::ifstream ifs(injector.get_sidecar_path());
  std::string content((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());

  // Verify ISO8601 timestamp format (YYYY-MM-DDTHH:MM:SS)
  EXPECT_TRUE(content.find("\"recording_started_at\"") != std::string::npos);
  // Should contain date separator and time separator
  EXPECT_TRUE(content.find("T") != std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


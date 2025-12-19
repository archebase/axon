#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <unistd.h>

#include "config_parser.hpp"

using namespace axon::core;

// =============================================================================
// Test Fixture
// =============================================================================

class ConfigParserTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temporary config file using filesystem
    auto temp_dir = std::filesystem::temp_directory_path();
    temp_config_ = (temp_dir / ("axon_test_config_" + std::to_string(getpid()) + ".yaml")).string();
    std::ofstream file(temp_config_);
    file << R"(
dataset:
  path: "/tmp/test_dataset.lance"
  mode: "create"

topics:
  - name: "/test/topic1"
    message_type: "std_msgs/String"
    batch_size: 50
    flush_interval_ms: 500
    
  - name: "/test/topic2"
    message_type: "std_msgs/Int32"
    batch_size: 100
    flush_interval_ms: 1000

recording:
  auto_start: false
  max_disk_usage_gb: 50.0
)";
    file.close();
  }

  void TearDown() override {
    if (std::filesystem::exists(temp_config_)) {
      std::filesystem::remove(temp_config_);
    }
  }

  std::string temp_config_;
};

// =============================================================================
// File Loading Tests
// =============================================================================

TEST_F(ConfigParserTest, LoadFromFile) {
  ConfigParser parser;
  RecorderConfig config;

  ASSERT_TRUE(parser.load_from_file(temp_config_, config));
  ASSERT_EQ(config.dataset.path, "/tmp/test_dataset.lance");
  ASSERT_EQ(config.dataset.mode, "create");
  ASSERT_EQ(config.topics.size(), 2);
  ASSERT_EQ(config.topics[0].name, "/test/topic1");
  ASSERT_EQ(config.topics[0].message_type, "std_msgs/String");
  ASSERT_EQ(config.topics[0].batch_size, 50);
  ASSERT_FALSE(config.recording.auto_start);
}

TEST_F(ConfigParserTest, LoadFromNonexistentFile) {
  ConfigParser parser;
  RecorderConfig config;

  ASSERT_FALSE(parser.load_from_file("/nonexistent/path/config.yaml", config));
}

// =============================================================================
// String Loading Tests
// =============================================================================

TEST_F(ConfigParserTest, LoadFromString) {
  ConfigParser parser;
  RecorderConfig config;

  std::string yaml_content = R"(
dataset:
  path: "/tmp/string_dataset.lance"
  mode: "append"

topics:
  - name: "/string/topic"
    message_type: "sensor_msgs/Image"
    batch_size: 10
    flush_interval_ms: 100

recording:
  auto_start: true
  max_disk_usage_gb: 100.0
)";

  ASSERT_TRUE(parser.load_from_string(yaml_content, config));
  EXPECT_EQ(config.dataset.path, "/tmp/string_dataset.lance");
  EXPECT_EQ(config.dataset.mode, "append");
  EXPECT_EQ(config.topics.size(), 1);
  EXPECT_EQ(config.topics[0].name, "/string/topic");
  EXPECT_TRUE(config.recording.auto_start);
}

TEST_F(ConfigParserTest, LoadFromEmptyString) {
  ConfigParser parser;
  RecorderConfig config;

  ASSERT_FALSE(parser.load_from_string("", config));
}

TEST_F(ConfigParserTest, LoadFromInvalidYaml) {
  ConfigParser parser;
  RecorderConfig config;

  std::string invalid_yaml = R"(
dataset:
  path: [invalid
  mode: also invalid
    bad indentation
)";

  ASSERT_FALSE(parser.load_from_string(invalid_yaml, config));
}

// =============================================================================
// Validation Tests
// =============================================================================

TEST_F(ConfigParserTest, ValidateConfig) {
  ConfigParser parser;
  RecorderConfig config;

  ASSERT_TRUE(parser.load_from_file(temp_config_, config));
  ASSERT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, InvalidConfigEmptyPath) {
  RecorderConfig config;
  config.dataset.path = "";  // Invalid: empty path
  config.topics.push_back({"/test", "std_msgs/String", 10, 100});

  ASSERT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, InvalidConfigEmptyTopics) {
  RecorderConfig config;
  config.dataset.path = "/tmp/test.lance";
  // No topics

  ASSERT_FALSE(config.validate());
}

TEST_F(ConfigParserTest, ValidateWithErrorMessage) {
  RecorderConfig config;
  config.dataset.path = "";
  config.topics.push_back({"/test", "std_msgs/String", 10, 100});

  std::string error_msg;
  ASSERT_FALSE(ConfigParser::validate(config, error_msg));
  EXPECT_FALSE(error_msg.empty());
}

TEST_F(ConfigParserTest, ValidConfigPassesValidation) {
  RecorderConfig config;
  config.dataset.path = "/tmp/valid.lance";
  config.dataset.mode = "create";
  config.topics.push_back({"/valid/topic", "std_msgs/String", 100, 1000});
  config.recording.auto_start = true;
  config.recording.max_disk_usage_gb = 50.0;

  std::string error_msg;
  ASSERT_TRUE(ConfigParser::validate(config, error_msg));
}

// =============================================================================
// Save Tests
// =============================================================================

TEST_F(ConfigParserTest, SaveToFile) {
  ConfigParser parser;
  RecorderConfig config;

  config.dataset.path = "/tmp/saved_dataset.lance";
  config.dataset.mode = "append";
  config.topics.push_back({"/saved/topic", "std_msgs/String", 25, 250});
  config.recording.auto_start = true;

  std::string save_path = temp_config_ + "_saved";
  ASSERT_TRUE(parser.save_to_file(save_path, config));

  // Reload and verify
  RecorderConfig loaded;
  ASSERT_TRUE(parser.load_from_file(save_path, loaded));
  ASSERT_EQ(loaded.dataset.path, config.dataset.path);
  ASSERT_EQ(loaded.topics.size(), 1);

  std::filesystem::remove(save_path);
}

TEST_F(ConfigParserTest, SaveMultipleTopics) {
  ConfigParser parser;
  RecorderConfig config;

  config.dataset.path = "/tmp/multi_topic.lance";
  config.dataset.mode = "create";
  config.topics.push_back({"/topic1", "std_msgs/String", 10, 100});
  config.topics.push_back({"/topic2", "sensor_msgs/Image", 20, 200});
  config.topics.push_back({"/topic3", "nav_msgs/Odometry", 30, 300});
  config.recording.auto_start = false;
  config.recording.max_disk_usage_gb = 25.5;

  std::string save_path = temp_config_ + "_multi";
  ASSERT_TRUE(parser.save_to_file(save_path, config));

  // Reload and verify all topics
  RecorderConfig loaded;
  ASSERT_TRUE(parser.load_from_file(save_path, loaded));
  ASSERT_EQ(loaded.topics.size(), 3);
  EXPECT_EQ(loaded.topics[0].name, "/topic1");
  EXPECT_EQ(loaded.topics[1].name, "/topic2");
  EXPECT_EQ(loaded.topics[2].name, "/topic3");
  EXPECT_EQ(loaded.topics[2].batch_size, 30);

  std::filesystem::remove(save_path);
}

// =============================================================================
// Static Methods Tests
// =============================================================================

TEST_F(ConfigParserTest, FromYamlFile) {
  auto config = RecorderConfig::from_yaml(temp_config_);

  EXPECT_EQ(config.dataset.path, "/tmp/test_dataset.lance");
  EXPECT_EQ(config.topics.size(), 2);
}

TEST_F(ConfigParserTest, FromYamlString) {
  std::string yaml = R"(
dataset:
  path: "/tmp/static_test.lance"
  mode: "create"

topics:
  - name: "/static/topic"
    message_type: "std_msgs/Int32"
    batch_size: 5
    flush_interval_ms: 50

recording:
  auto_start: true
  max_disk_usage_gb: 10.0
)";

  auto config = RecorderConfig::from_yaml_string(yaml);

  EXPECT_EQ(config.dataset.path, "/tmp/static_test.lance");
  EXPECT_EQ(config.topics[0].batch_size, 5);
}

TEST_F(ConfigParserTest, ConfigToString) {
  RecorderConfig config;
  config.dataset.path = "/tmp/to_string.lance";
  config.dataset.mode = "append";
  config.topics.push_back({"/test", "std_msgs/String", 100, 1000});
  config.recording.auto_start = true;

  std::string str = config.to_string();

  // Should contain key information
  EXPECT_NE(str.find("/tmp/to_string.lance"), std::string::npos);
  EXPECT_NE(str.find("/test"), std::string::npos);
}

// =============================================================================
// Default Values Tests
// =============================================================================

TEST_F(ConfigParserTest, DefaultValues) {
  TopicConfig topic;

  // Check defaults
  EXPECT_EQ(topic.batch_size, 100);
  EXPECT_EQ(topic.flush_interval_ms, 1000);

  DatasetConfig dataset;
  EXPECT_EQ(dataset.mode, "append");

  RecordingConfig recording;
  EXPECT_TRUE(recording.auto_start);
  EXPECT_DOUBLE_EQ(recording.max_disk_usage_gb, 100.0);
}

TEST_F(ConfigParserTest, PartialConfigUsesDefaults) {
  ConfigParser parser;
  RecorderConfig config;

  std::string partial_yaml = R"(
dataset:
  path: "/tmp/partial.lance"

topics:
  - name: "/partial/topic"
    message_type: "std_msgs/String"
)";

  ASSERT_TRUE(parser.load_from_string(partial_yaml, config));

  // Should use defaults for missing fields
  EXPECT_EQ(config.dataset.mode, "append");
  EXPECT_EQ(config.topics[0].batch_size, 100);
  EXPECT_EQ(config.topics[0].flush_interval_ms, 1000);
}

// =============================================================================
// Edge Cases Tests
// =============================================================================

TEST_F(ConfigParserTest, SpecialCharactersInPath) {
  ConfigParser parser;
  RecorderConfig config;

  std::string yaml = R"(
dataset:
  path: "/tmp/special chars/path with spaces.lance"
  mode: "create"

topics:
  - name: "/topic/with-dashes_and_underscores"
    message_type: "std_msgs/String"
    batch_size: 10
    flush_interval_ms: 100

recording:
  auto_start: true
)";

  ASSERT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.dataset.path, "/tmp/special chars/path with spaces.lance");
  EXPECT_EQ(config.topics[0].name, "/topic/with-dashes_and_underscores");
}

TEST_F(ConfigParserTest, LargeValues) {
  ConfigParser parser;
  RecorderConfig config;

  std::string yaml = R"(
dataset:
  path: "/tmp/large.lance"
  mode: "create"

topics:
  - name: "/large/topic"
    message_type: "std_msgs/String"
    batch_size: 1000000
    flush_interval_ms: 60000

recording:
  auto_start: false
  max_disk_usage_gb: 10000.0
)";

  ASSERT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.topics[0].batch_size, 1000000);
  EXPECT_EQ(config.topics[0].flush_interval_ms, 60000);
  EXPECT_DOUBLE_EQ(config.recording.max_disk_usage_gb, 10000.0);
}

TEST_F(ConfigParserTest, ManyTopics) {
  ConfigParser parser;
  RecorderConfig config;

  std::string yaml = R"(
dataset:
  path: "/tmp/many.lance"
  mode: "create"

topics:
)";

  // Add 20 topics
  for (int i = 0; i < 20; ++i) {
    yaml += "  - name: \"/topic" + std::to_string(i) + "\"\n";
    yaml += "    message_type: \"std_msgs/String\"\n";
    yaml += "    batch_size: " + std::to_string(i + 1) + "\n";
    yaml += "    flush_interval_ms: " + std::to_string((i + 1) * 10) + "\n";
  }

  yaml += R"(
recording:
  auto_start: true
)";

  ASSERT_TRUE(parser.load_from_string(yaml, config));
  EXPECT_EQ(config.topics.size(), 20);
  EXPECT_EQ(config.topics[19].name, "/topic19");
  EXPECT_EQ(config.topics[19].batch_size, 20);
}

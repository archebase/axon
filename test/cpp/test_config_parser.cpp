#include <gtest/gtest.h>
#include "../src/cpp/core/config_parser.hpp"
#include <fstream>
#include <filesystem>

using namespace lance_recorder::core;

class ConfigParserTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create temporary config file
        temp_config_ = std::tmpnam(nullptr);
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

TEST_F(ConfigParserTest, ValidateConfig) {
    ConfigParser parser;
    RecorderConfig config;
    
    ASSERT_TRUE(parser.load_from_file(temp_config_, config));
    ASSERT_TRUE(config.validate());
}

TEST_F(ConfigParserTest, InvalidConfig) {
    RecorderConfig config;
    config.dataset.path = "";  // Invalid: empty path
    config.topics.push_back({"/test", "std_msgs/String", 10, 100});
    
    ASSERT_FALSE(config.validate());
}

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


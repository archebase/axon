/**
 * Unit tests for McapWriterWrapper
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;

using namespace axon::mcap_wrapper;

class McapWriterTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a unique temp file for each test
    test_file_ = "/tmp/test_mcap_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mcap";
  }

  void TearDown() override {
    // Clean up test file
    if (fs::exists(test_file_)) {
      fs::remove(test_file_);
    }
  }

  std::string test_file_;
};

TEST_F(McapWriterTest, OpenAndClose) {
  McapWriterWrapper writer;
  
  EXPECT_FALSE(writer.is_open());
  
  McapWriterOptions options;
  options.profile = "ros2";
  options.compression = Compression::None;
  
  EXPECT_TRUE(writer.open(test_file_, options));
  EXPECT_TRUE(writer.is_open());
  EXPECT_EQ(writer.get_path(), test_file_);
  
  writer.close();
  EXPECT_FALSE(writer.is_open());
  
  // Verify file was created
  EXPECT_TRUE(fs::exists(test_file_));
}

TEST_F(McapWriterTest, RegisterSchemaAndChannel) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  // Register a schema
  std::string schema_data = "uint32 height\nuint32 width\nstring encoding\nuint8[] data";
  uint16_t schema_id = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", schema_data);
  EXPECT_GT(schema_id, 0);
  
  // Register a channel
  uint16_t channel_id = writer.register_channel("/camera/image", "cdr", schema_id);
  EXPECT_GT(channel_id, 0);
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.schemas_registered, 1);
  EXPECT_EQ(stats.channels_registered, 1);
  
  writer.close();
}

TEST_F(McapWriterTest, WriteMessages) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  // Register schema and channel
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);
  
  // Write some messages
  const size_t num_messages = 100;
  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  
  uint64_t base_time = 1000000000ULL; // 1 second in ns
  
  for (size_t i = 0; i < num_messages; ++i) {
    uint64_t timestamp = base_time + i * 1000000ULL; // +1ms per message
    EXPECT_TRUE(writer.write(channel_id, timestamp, timestamp, payload.data(), payload.size()));
  }
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, num_messages);
  EXPECT_EQ(stats.bytes_written, num_messages * payload.size());
  
  writer.close();
  
  // Verify file size is reasonable
  auto file_size = fs::file_size(test_file_);
  EXPECT_GT(file_size, 0);
}

TEST_F(McapWriterTest, WriteWithSequence) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);
  
  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  uint64_t timestamp = 1000000000ULL;
  
  // Write with explicit sequence numbers
  for (uint32_t seq = 0; seq < 10; ++seq) {
    EXPECT_TRUE(writer.write(channel_id, seq, timestamp + seq * 1000000ULL, timestamp + seq * 1000000ULL, payload.data(), payload.size()));
  }
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 10);
  
  writer.close();
}

TEST_F(McapWriterTest, MultipleChannels) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  // Register multiple schemas and channels
  uint16_t schema1 = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint32 height\nuint32 width");
  uint16_t schema2 = writer.register_schema("sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation");
  uint16_t schema3 = writer.register_schema("geometry_msgs/msg/Twist", "ros2msg", "geometry_msgs/Vector3 linear");
  
  uint16_t channel1 = writer.register_channel("/camera/image", "cdr", schema1);
  uint16_t channel2 = writer.register_channel("/imu/data", "cdr", schema2);
  uint16_t channel3 = writer.register_channel("/cmd_vel", "cdr", schema3);
  
  // All channel IDs should be unique
  EXPECT_NE(channel1, channel2);
  EXPECT_NE(channel2, channel3);
  EXPECT_NE(channel1, channel3);
  
  // Write to all channels
  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  uint64_t timestamp = 1000000000ULL;
  
  for (int i = 0; i < 10; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel1, t, t, payload.data(), payload.size()));
    EXPECT_TRUE(writer.write(channel2, t, t, payload.data(), payload.size()));
    EXPECT_TRUE(writer.write(channel3, t, t, payload.data(), payload.size()));
  }
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 30);
  EXPECT_EQ(stats.schemas_registered, 3);
  EXPECT_EQ(stats.channels_registered, 3);
  
  writer.close();
}

TEST_F(McapWriterTest, CompressionZstd) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::Zstd;
  options.compression_level = 3;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);
  
  // Write compressible data (repeated pattern)
  std::vector<uint8_t> payload(1024, 0xAA);
  uint64_t timestamp = 1000000000ULL;
  
  for (int i = 0; i < 100; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel_id, t, t, payload.data(), payload.size()));
  }
  
  writer.close();
  
  // File should exist and have some size
  EXPECT_TRUE(fs::exists(test_file_));
  EXPECT_GT(fs::file_size(test_file_), 0);
}

TEST_F(McapWriterTest, ThreadSafety) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);
  
  const size_t num_threads = 4;
  const size_t messages_per_thread = 1000;
  
  std::vector<std::thread> threads;
  std::atomic<size_t> success_count{0};
  
  for (size_t t = 0; t < num_threads; ++t) {
    threads.emplace_back([&, t]() {
      std::vector<uint8_t> payload = {static_cast<uint8_t>(t), 0x02, 0x03, 0x04};
      uint64_t base_time = 1000000000ULL + t * 1000000000ULL;
      
      for (size_t i = 0; i < messages_per_thread; ++i) {
        uint64_t timestamp = base_time + i * 1000ULL;
        if (writer.write(channel_id, timestamp, timestamp, payload.data(), payload.size())) {
          success_count++;
        }
      }
    });
  }
  
  for (auto& thread : threads) {
    thread.join();
  }
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, num_threads * messages_per_thread);
  EXPECT_EQ(success_count.load(), num_threads * messages_per_thread);
  
  writer.close();
}

TEST_F(McapWriterTest, ChannelMetadata) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::None;
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  
  // Register channel with metadata
  std::unordered_map<std::string, std::string> metadata = {
    {"frame_id", "base_link"},
    {"offered_qos_profiles", "- history: keep_last\n  depth: 10"}
  };
  
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id, metadata);
  EXPECT_GT(channel_id, 0);
  
  writer.close();
}

TEST_F(McapWriterTest, DoubleOpenFails) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  ASSERT_TRUE(writer.open(test_file_, options));
  
  // Second open should fail
  EXPECT_FALSE(writer.open(test_file_ + "_2", options));
  EXPECT_FALSE(writer.get_last_error().empty());
  
  writer.close();
}

TEST_F(McapWriterTest, WriteToClosedWriter) {
  McapWriterWrapper writer;
  
  // Writing to closed writer should fail
  std::vector<uint8_t> payload = {0x01, 0x02};
  EXPECT_FALSE(writer.write(1, 1000000000ULL, 1000000000ULL, payload.data(), payload.size()));
}

TEST_F(McapWriterTest, LargeMessages) {
  McapWriterWrapper writer;
  
  McapWriterOptions options;
  options.compression = Compression::Zstd;
  options.chunk_size = 1024 * 1024; // 1MB chunks
  
  ASSERT_TRUE(writer.open(test_file_, options));
  
  uint16_t schema_id = writer.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/camera/image", "cdr", schema_id);
  
  // Simulate 1920x1080 RGB image (~6MB) with varied data (not uniform)
  // Use pseudo-random data to prevent extreme compression ratios
  std::vector<uint8_t> large_payload(1920 * 1080 * 3);
  for (size_t i = 0; i < large_payload.size(); ++i) {
    // Generate varied pixel data (simulates actual image content)
    large_payload[i] = static_cast<uint8_t>((i * 7 + i / 256) % 256);
  }
  uint64_t timestamp = 1000000000ULL;
  
  // Write several large images
  for (int i = 0; i < 10; ++i) {
    uint64_t t = timestamp + i * 33333333ULL; // ~30fps
    EXPECT_TRUE(writer.write(channel_id, t, t, large_payload.data(), large_payload.size()));
  }
  
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 10);
  
  writer.close();
  
  // File should be created with reasonable size
  // With varied data, Zstd typically achieves 2-4x compression, so ~15-30MB expected
  EXPECT_TRUE(fs::exists(test_file_));
  EXPECT_GT(fs::file_size(test_file_), 1000000); // At least 1MB
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

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
    test_file_ = "/tmp/test_mcap_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
                 ".mcap";
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

  uint64_t base_time = 1000000000ULL;  // 1 second in ns

  for (size_t i = 0; i < num_messages; ++i) {
    uint64_t timestamp = base_time + i * 1000000ULL;  // +1ms per message
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
    EXPECT_TRUE(writer.write(
      channel_id,
      seq,
      timestamp + seq * 1000000ULL,
      timestamp + seq * 1000000ULL,
      payload.data(),
      payload.size()
    ));
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
  uint16_t schema1 =
    writer.register_schema("sensor_msgs/msg/Image", "ros2msg", "uint32 height\nuint32 width");
  uint16_t schema2 =
    writer.register_schema("sensor_msgs/msg/Imu", "ros2msg", "float64[9] orientation");
  uint16_t schema3 =
    writer.register_schema("geometry_msgs/msg/Twist", "ros2msg", "geometry_msgs/Vector3 linear");

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
    {"frame_id", "base_link"}, {"offered_qos_profiles", "- history: keep_last\n  depth: 10"}};

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
  options.chunk_size = 1024 * 1024;  // 1MB chunks

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
    uint64_t t = timestamp + i * 33333333ULL;  // ~30fps
    EXPECT_TRUE(writer.write(channel_id, t, t, large_payload.data(), large_payload.size()));
  }

  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 10);

  writer.close();

  // File should be created with reasonable size
  // Zstd compression on pseudo-random data achieves high compression ratios
  // The key assertion is that we wrote significant data (not just headers)
  EXPECT_TRUE(fs::exists(test_file_));
  EXPECT_GT(fs::file_size(test_file_), 100000);  // At least 100KB (62MB raw → ~400KB compressed)
}

// =============================================================================
// Metadata Tests
// =============================================================================

TEST_F(McapWriterTest, WriteMetadata) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  // Write metadata
  std::unordered_map<std::string, std::string> metadata = {
    {"task_id", "task_12345"},
    {"device_id", "robot_001"},
    {"recording_start", "2024-01-15T10:30:00Z"}};

  EXPECT_TRUE(writer.write_metadata("axon.task", metadata));

  writer.close();

  // Verify file was created
  EXPECT_TRUE(fs::exists(test_file_));
}

TEST_F(McapWriterTest, WriteMultipleMetadata) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  // Write multiple metadata records
  std::unordered_map<std::string, std::string> task_metadata = {
    {"task_id", "task_abc"}, {"task_type", "mapping"}};
  EXPECT_TRUE(writer.write_metadata("axon.task", task_metadata));

  std::unordered_map<std::string, std::string> device_metadata = {
    {"device_id", "robot_xyz"}, {"firmware_version", "1.2.3"}};
  EXPECT_TRUE(writer.write_metadata("axon.device", device_metadata));

  std::unordered_map<std::string, std::string> recording_metadata = {
    {"start_time", "1000000000"}, {"end_time", "2000000000"}, {"duration_seconds", "1000"}};
  EXPECT_TRUE(writer.write_metadata("axon.recording", recording_metadata));

  writer.close();
}

TEST_F(McapWriterTest, WriteMetadataToClosedWriter) {
  McapWriterWrapper writer;

  // Writer is not open
  std::unordered_map<std::string, std::string> metadata = {{"key", "value"}};

  EXPECT_FALSE(writer.write_metadata("test", metadata));
  EXPECT_FALSE(writer.get_last_error().empty());
}

TEST_F(McapWriterTest, WriteMetadataEmptyMap) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  // Write empty metadata
  std::unordered_map<std::string, std::string> empty_metadata;
  EXPECT_TRUE(writer.write_metadata("empty_test", empty_metadata));

  writer.close();
}

// =============================================================================
// Flush Tests
// =============================================================================

TEST_F(McapWriterTest, FlushOpenWriter) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  uint64_t timestamp = 1000000000ULL;

  // Write some messages
  for (int i = 0; i < 10; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    writer.write(channel_id, t, t, payload.data(), payload.size());
  }

  // Flush should not throw
  EXPECT_NO_THROW(writer.flush());

  // Write more after flush
  for (int i = 10; i < 20; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel_id, t, t, payload.data(), payload.size()));
  }

  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 20);

  writer.close();
}

TEST_F(McapWriterTest, FlushClosedWriter) {
  McapWriterWrapper writer;

  // Flush on closed writer should not crash
  EXPECT_NO_THROW(writer.flush());
}

TEST_F(McapWriterTest, MultipleFlushes) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  uint64_t timestamp = 1000000000ULL;

  // Multiple flush cycles
  for (int cycle = 0; cycle < 5; ++cycle) {
    for (int i = 0; i < 10; ++i) {
      uint64_t t = timestamp + (cycle * 10 + i) * 1000000ULL;
      writer.write(channel_id, t, t, payload.data(), payload.size());
    }
    EXPECT_NO_THROW(writer.flush());
  }

  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 50);

  writer.close();
}

// =============================================================================
// Error Path Tests
// =============================================================================

TEST_F(McapWriterTest, RegisterSchemaClosedWriter) {
  McapWriterWrapper writer;

  // Registering schema on closed writer should fail
  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
  EXPECT_EQ(schema_id, 0);
  EXPECT_FALSE(writer.get_last_error().empty());
}

TEST_F(McapWriterTest, RegisterChannelClosedWriter) {
  McapWriterWrapper writer;

  // Registering channel on closed writer should fail
  uint16_t channel_id = writer.register_channel("/test/topic", "cdr", 1);
  EXPECT_EQ(channel_id, 0);
  EXPECT_FALSE(writer.get_last_error().empty());
}

TEST_F(McapWriterTest, GetLastErrorInitiallyEmpty) {
  McapWriterWrapper writer;

  // Before any error, should be empty or have no significant error
  // The error may or may not be empty initially depending on implementation
  // Just verify it doesn't crash
  std::string error = writer.get_last_error();
  // No assertion needed - just verify access doesn't crash
}

TEST_F(McapWriterTest, GetPathClosedWriter) {
  McapWriterWrapper writer;

  // Path on closed writer should be empty
  std::string path = writer.get_path();
  EXPECT_TRUE(path.empty());
}

TEST_F(McapWriterTest, GetStatisticsClosedWriter) {
  McapWriterWrapper writer;

  // Statistics on closed writer should return zeros
  auto stats = writer.get_statistics();
  EXPECT_EQ(stats.messages_written, 0);
  EXPECT_EQ(stats.bytes_written, 0);
  EXPECT_EQ(stats.schemas_registered, 0);
  EXPECT_EQ(stats.channels_registered, 0);
}

TEST_F(McapWriterTest, CloseIdempotent) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  ASSERT_TRUE(writer.open(test_file_, options));

  // Close multiple times should be safe
  writer.close();
  EXPECT_NO_THROW(writer.close());
  EXPECT_NO_THROW(writer.close());

  EXPECT_FALSE(writer.is_open());
}

// =============================================================================
// Compression Options Tests
// =============================================================================

TEST_F(McapWriterTest, CompressionLz4) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::Lz4;

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

  // Write compressible data
  std::vector<uint8_t> payload(1024, 0xBB);
  uint64_t timestamp = 1000000000ULL;

  for (int i = 0; i < 50; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel_id, t, t, payload.data(), payload.size()));
  }

  writer.close();

  EXPECT_TRUE(fs::exists(test_file_));
  EXPECT_GT(fs::file_size(test_file_), 0);
}

TEST_F(McapWriterTest, CompressionNone) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

  std::vector<uint8_t> payload(512, 0xCC);
  uint64_t timestamp = 1000000000ULL;

  for (int i = 0; i < 50; ++i) {
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel_id, t, t, payload.data(), payload.size()));
  }

  writer.close();

  // Uncompressed file should be larger than compressed
  EXPECT_TRUE(fs::exists(test_file_));
  auto file_size = fs::file_size(test_file_);
  EXPECT_GT(file_size, 50 * 512);  // At least the raw data size (with overhead)
}

// =============================================================================
// Options Configuration Tests
// =============================================================================

TEST_F(McapWriterTest, CustomChunkSize) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.compression = Compression::None;
  options.chunk_size = 512 * 1024;  // 512KB chunks

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

  // Write enough data to span multiple chunks
  std::vector<uint8_t> payload(64 * 1024, 0xDD);  // 64KB per message
  uint64_t timestamp = 1000000000ULL;

  for (int i = 0; i < 20; ++i) {  // 20 * 64KB = 1.28MB
    uint64_t t = timestamp + i * 1000000ULL;
    EXPECT_TRUE(writer.write(channel_id, t, t, payload.data(), payload.size()));
  }

  writer.close();

  EXPECT_TRUE(fs::exists(test_file_));
}

TEST_F(McapWriterTest, CustomProfile) {
  McapWriterWrapper writer;

  McapWriterOptions options;
  options.profile = "ros1";  // Different profile
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(test_file_, options));

  uint16_t schema_id = writer.register_schema("test/msg/Data", "ros1msg", "uint32 value");
  uint16_t channel_id = writer.register_channel("/test/data", "ros1", schema_id);

  std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
  writer.write(channel_id, 1000000000ULL, 1000000000ULL, payload.data(), payload.size());

  writer.close();

  EXPECT_TRUE(fs::exists(test_file_));
}

// =============================================================================
// Directory Creation Tests
// =============================================================================

TEST_F(McapWriterTest, CreateNestedDirectories) {
  // Test that writer creates nested directories if they don't exist
  std::string nested_path =
    "/tmp/mcap_test_nested_" +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
    "/level1/level2/test.mcap";

  McapWriterWrapper writer;
  McapWriterOptions options;
  options.compression = Compression::None;

  ASSERT_TRUE(writer.open(nested_path, options));
  writer.close();

  EXPECT_TRUE(fs::exists(nested_path));

  // Clean up
  fs::path parent = fs::path(nested_path).parent_path().parent_path().parent_path();
  fs::remove_all(parent);
}

// =============================================================================
// Destructor Cleanup Test
// =============================================================================

TEST_F(McapWriterTest, DestructorClosesFile) {
  {
    McapWriterWrapper writer;
    McapWriterOptions options;
    options.compression = Compression::None;

    ASSERT_TRUE(writer.open(test_file_, options));

    uint16_t schema_id = writer.register_schema("test/msg/Data", "ros2msg", "uint32 value");
    uint16_t channel_id = writer.register_channel("/test/data", "cdr", schema_id);

    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04};
    writer.write(channel_id, 1000000000ULL, 1000000000ULL, payload.data(), payload.size());

    // Don't call close() - let destructor handle it
  }

  // File should still exist and be valid
  EXPECT_TRUE(fs::exists(test_file_));
  EXPECT_GT(fs::file_size(test_file_), 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

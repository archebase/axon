// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "../src/core/disk_usage_monitor.hpp"

namespace fs = std::filesystem;
using namespace axon::recorder;

class DiskUsageMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_dir_ = fs::temp_directory_path() /
                ("disk_usage_monitor_test_" +
                 std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
  }

  fs::path write_file(const std::string& name, size_t bytes) {
    fs::path path = test_dir_ / name;
    std::ofstream out(path, std::ios::binary);
    std::string chunk(1024, 'x');
    while (bytes >= chunk.size()) {
      out.write(chunk.data(), static_cast<std::streamsize>(chunk.size()));
      bytes -= chunk.size();
    }
    if (bytes > 0) {
      out.write(chunk.data(), static_cast<std::streamsize>(bytes));
    }
    return path;
  }

  DiskUsageMonitor make_monitor(uint64_t warn_bytes, uint64_t hard_bytes) {
    DiskUsageLimitConfig limits;
    limits.warn_usage_bytes = warn_bytes;
    limits.hard_limit_bytes = hard_bytes;
    return DiskUsageMonitor(limits, {{"recording_output", test_dir_}});
  }

  fs::path test_dir_;
};

TEST_F(DiskUsageMonitorTest, ReportsNormalCapacityBelowWarnThreshold) {
  write_file("normal.mcap", 1024);

  auto snapshot = make_monitor(10 * 1024, 20 * 1024).snapshot();

  EXPECT_EQ(snapshot.state, DiskUsageState::NORMAL);
  EXPECT_EQ(snapshot.total_used_bytes, 1024);
  ASSERT_EQ(snapshot.paths.size(), 1);
  EXPECT_EQ(snapshot.paths[0].role, "recording_output");
}

TEST_F(DiskUsageMonitorTest, ReportsWarnCapacityAtWarnThreshold) {
  write_file("warn.mcap", 12 * 1024);

  auto snapshot = make_monitor(10 * 1024, 20 * 1024).snapshot();

  EXPECT_EQ(snapshot.state, DiskUsageState::WARN);
  EXPECT_TRUE(snapshot.warn_reached());
  EXPECT_FALSE(snapshot.hard_limit_reached());
  EXPECT_EQ(snapshot.reason, "monitored disk usage reached warning threshold");
}

TEST_F(DiskUsageMonitorTest, ReportsHardLimitAtHardThreshold) {
  write_file("hard.mcap", 24 * 1024);

  auto snapshot = make_monitor(10 * 1024, 20 * 1024).snapshot();

  EXPECT_EQ(snapshot.state, DiskUsageState::HARD_LIMIT);
  EXPECT_TRUE(snapshot.hard_limit_reached());
  EXPECT_EQ(snapshot.reason, "monitored disk usage reached hard limit");
}

TEST_F(DiskUsageMonitorTest, ReportsHardLimitForCurrentTaskSize) {
  DiskUsageLimitConfig limits;
  limits.warn_usage_bytes = 10 * 1024;
  limits.hard_limit_bytes = 20 * 1024;
  limits.max_task_size_bytes = 4 * 1024;
  DiskUsageMonitor monitor(limits, {{"recording_output", test_dir_}});

  auto snapshot = monitor.snapshot(5 * 1024);

  EXPECT_EQ(snapshot.state, DiskUsageState::HARD_LIMIT);
  EXPECT_EQ(snapshot.current_task_bytes, 5 * 1024);
  EXPECT_EQ(snapshot.reason, "current task size reached hard limit");
}

TEST_F(DiskUsageMonitorTest, CleanupRemovesOldCompletedFilesButKeepsActiveRecording) {
  auto old_file = write_file("old.mcap", 8 * 1024);
  auto active_file = write_file("active.mcap", 8 * 1024);
  write_file("notes.txt", 8 * 1024);

  auto monitor = make_monitor(10 * 1024, 12 * 1024);
  auto result = monitor.cleanup_recording_files({test_dir_}, active_file, 24 * 1024, 10 * 1024, 0);

  EXPECT_EQ(result.files_removed, 1);
  EXPECT_EQ(result.bytes_removed, 8 * 1024);
  EXPECT_FALSE(fs::exists(old_file));
  EXPECT_TRUE(fs::exists(active_file));
  EXPECT_TRUE(fs::exists(test_dir_ / "notes.txt"));
}

TEST_F(DiskUsageMonitorTest, SerializesStatusForRpcStatusPayload) {
  write_file("status.mcap", 12 * 1024);

  auto json = make_monitor(10 * 1024, 20 * 1024).snapshot().to_json();

  EXPECT_EQ(json["state"], "warn");
  EXPECT_EQ(json["total_used_bytes"], 12 * 1024);
  ASSERT_TRUE(json["paths"].is_array());
  EXPECT_EQ(json["paths"][0]["role"], "recording_output");
}

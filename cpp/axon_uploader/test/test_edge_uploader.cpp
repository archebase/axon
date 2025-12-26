/**
 * Integration tests for EdgeUploader
 *
 * These tests require MinIO to be running locally:
 *
 *   # 1. Create tmpfs mount (RAM disk)
 *   sudo mkdir -p /mnt/minio-tmpfs
 *   sudo mount -t tmpfs -o size=2G tmpfs /mnt/minio-tmpfs
 *
 *   # 2. Run MinIO
 *   docker run -d --name minio-dev \
 *     -p 9000:9000 -p 9001:9001 \
 *     -v /mnt/minio-tmpfs:/data \
 *     -e "MINIO_ROOT_USER=minioadmin" \
 *     -e "MINIO_ROOT_PASSWORD=minioadmin" \
 *     quay.io/minio/minio server /data --console-address ":9001"
 *
 *   # 3. Create test bucket
 *   docker run --rm --network host \
 *     -e MC_HOST_local=http://minioadmin:minioadmin@localhost:9000 \
 *     quay.io/minio/mc mb local/axon-raw-data
 *
 *   # 4. Set environment variables and run test
 *   export AWS_ACCESS_KEY_ID=minioadmin
 *   export AWS_SECRET_ACCESS_KEY=minioadmin
 *   ./test_edge_uploader
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <thread>

#include "edge_uploader.hpp"

namespace fs = std::filesystem;
using namespace axon::uploader;

// Check if MinIO is available
bool isMinIOAvailable() {
  // Check for required environment variables
  if (!std::getenv("AWS_ACCESS_KEY_ID") || !std::getenv("AWS_SECRET_ACCESS_KEY")) {
    return false;
  }

  // Try to connect to MinIO endpoint
  // For this simple test, just check env vars are set
  return true;
}

class EdgeUploaderIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!isMinIOAvailable()) {
      GTEST_SKIP() << "MinIO not available - skipping integration test";
    }

    // Create test directory
    test_dir_ = "/tmp/axon_uploader_test_" +
                std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    fs::create_directories(test_dir_);

    // Create state DB path
    db_path_ = test_dir_ + "/state.db";

    // Setup uploader config
    config_.s3.endpoint_url = "http://localhost:9000";
    config_.s3.bucket = "axon-raw-data";
    config_.s3.region = "us-east-1";
    config_.s3.use_ssl = false;
    config_.s3.verify_ssl = false;

    config_.retry.max_retries = 3;
    config_.retry.initial_delay = std::chrono::milliseconds(100);
    config_.retry.max_delay = std::chrono::milliseconds(1000);

    config_.num_workers = 1;
    config_.state_db_path = db_path_;
    config_.delete_after_upload = false;  // Don't delete for test verification
    config_.failed_uploads_dir = test_dir_ + "/failed/";
  }

  void TearDown() override {
    // Cleanup test directory
    std::error_code ec;
    fs::remove_all(test_dir_, ec);

    // Cleanup state DB WAL files
    fs::remove(db_path_ + "-wal", ec);
    fs::remove(db_path_ + "-shm", ec);
  }

  // Create a test MCAP file (minimal valid structure)
  std::string createTestMcapFile(const std::string& name, size_t size = 1024) {
    std::string path = test_dir_ + "/" + name + ".mcap";

    std::ofstream file(path, std::ios::binary);
    if (!file) {
      return "";
    }

    // Write MCAP header magic
    static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
    file.write(MAGIC, 8);

    // Write padding to reach desired size
    if (size > 16) {
      std::vector<char> padding(size - 16, 0);
      file.write(padding.data(), padding.size());
    }

    // Write MCAP footer magic
    file.write(MAGIC, 8);

    file.close();
    return path;
  }

  // Create a test JSON sidecar file
  std::string createTestJsonFile(const std::string& name, const std::string& task_id) {
    std::string path = test_dir_ + "/" + name + ".json";

    std::ofstream file(path);
    if (!file) {
      return "";
    }

    file << R"({
      "task_id": ")" << task_id << R"(",
      "device_id": "test_device",
      "factory_id": "test_factory",
      "checksum_sha256": "abc123",
      "file_size_bytes": 1024
    })";

    file.close();
    return path;
  }

  std::string test_dir_;
  std::string db_path_;
  UploaderConfig config_;
};

TEST_F(EdgeUploaderIntegrationTest, StartStop) {
  EdgeUploader uploader(config_);

  EXPECT_FALSE(uploader.isRunning());

  uploader.start();
  EXPECT_TRUE(uploader.isRunning());

  uploader.stop();
  EXPECT_FALSE(uploader.isRunning());
}

TEST_F(EdgeUploaderIntegrationTest, HealthStatus) {
  EdgeUploader uploader(config_);
  uploader.start();

  auto health = uploader.getHealthStatus();
  EXPECT_TRUE(health.healthy);
  EXPECT_EQ(health.pending_count, 0);
  EXPECT_EQ(health.failed_count, 0);

  uploader.stop();
}

TEST_F(EdgeUploaderIntegrationTest, EnqueueAndUpload) {
  EdgeUploader uploader(config_);
  uploader.start();

  // Create test files
  std::string task_id = "test_task_001";
  std::string mcap_path = createTestMcapFile(task_id, 1024);
  std::string json_path = createTestJsonFile(task_id, task_id);

  ASSERT_FALSE(mcap_path.empty());
  ASSERT_FALSE(json_path.empty());
  ASSERT_TRUE(fs::exists(mcap_path));
  ASSERT_TRUE(fs::exists(json_path));

  // Track upload completion
  std::atomic<bool> upload_completed{false};
  std::string upload_error;

  uploader.setCallback([&](const std::string& id, bool success, const std::string& error) {
    if (id == task_id) {
      upload_completed = true;
      if (!success) {
        upload_error = error;
      }
    }
  });

  // Enqueue upload
  uploader.enqueue(mcap_path, json_path, task_id, "test_factory", "test_device", "checksum123");

  // Wait for upload (with timeout)
  auto start = std::chrono::steady_clock::now();
  while (!upload_completed &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(upload_completed) << "Upload did not complete within timeout";
  EXPECT_TRUE(upload_error.empty()) << "Upload failed: " << upload_error;

  // Verify stats
  const auto& stats = uploader.stats();
  EXPECT_EQ(stats.files_completed, 1);
  EXPECT_EQ(stats.files_failed, 0);

  uploader.stop();
}

TEST_F(EdgeUploaderIntegrationTest, CrashRecovery) {
  std::string task_id = "recovery_task_001";

  // Create test files
  std::string mcap_path = createTestMcapFile(task_id, 1024);
  std::string json_path = createTestJsonFile(task_id, task_id);

  ASSERT_FALSE(mcap_path.empty());
  ASSERT_FALSE(json_path.empty());

  // First session: enqueue but don't wait for completion
  {
    EdgeUploader uploader(config_);
    uploader.start();
    uploader.enqueue(mcap_path, json_path, task_id, "factory", "device", "checksum");

    // Stop without waiting (simulates crash)
    uploader.stop();
  }

  // Second session: should recover incomplete upload
  std::atomic<bool> upload_completed{false};

  {
    EdgeUploader uploader(config_);

    uploader.setCallback([&](const std::string& id, bool success, const std::string& /*error*/) {
      if (id == task_id && success) {
        upload_completed = true;
      }
    });

    uploader.start();  // This triggers crash recovery

    // Wait for recovery upload
    auto start = std::chrono::steady_clock::now();
    while (!upload_completed &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    uploader.stop();
  }

  EXPECT_TRUE(upload_completed) << "Crash recovery did not complete upload";
}

TEST_F(EdgeUploaderIntegrationTest, RetryOnFailure) {
  // Configure uploader with an invalid endpoint to force failures
  config_.s3.endpoint_url = "http://localhost:19999";  // Wrong port
  config_.s3.connect_timeout_ms = 1000;  // 1 second timeout for faster test failures
  // Note: max_sdk_retries defaults to 0, so EdgeUploader handles all retry logic
  config_.retry.max_retries = 2;
  config_.retry.initial_delay = std::chrono::milliseconds(50);

  EdgeUploader uploader(config_);
  uploader.start();

  std::string task_id = "retry_task_001";
  std::string mcap_path = createTestMcapFile(task_id, 1024);
  std::string json_path = createTestJsonFile(task_id, task_id);

  std::atomic<bool> upload_failed{false};

  uploader.setCallback([&](const std::string& id, bool success, const std::string& /*error*/) {
    if (id == task_id && !success) {
      upload_failed = true;
    }
  });

  uploader.enqueue(mcap_path, json_path, task_id, "factory", "device", "checksum");

  // Wait for failure (should happen after retries)
  auto start = std::chrono::steady_clock::now();
  while (!upload_failed &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(upload_failed) << "Upload should have failed";

  const auto& stats = uploader.stats();
  EXPECT_EQ(stats.files_failed, 1);

  uploader.stop();
}

TEST_F(EdgeUploaderIntegrationTest, MultipleUploads) {
  EdgeUploader uploader(config_);
  uploader.start();

  constexpr int num_files = 5;
  std::atomic<int> completed_count{0};

  uploader.setCallback([&](const std::string& /*id*/, bool success, const std::string& /*error*/) {
    if (success) {
      completed_count++;
    }
  });

  // Enqueue multiple files
  for (int i = 0; i < num_files; ++i) {
    std::string task_id = "multi_task_" + std::to_string(i);
    std::string mcap_path = createTestMcapFile(task_id, 512);
    std::string json_path = createTestJsonFile(task_id, task_id);

    uploader.enqueue(mcap_path, json_path, task_id, "factory", "device", "checksum");
  }

  // Wait for all uploads
  auto start = std::chrono::steady_clock::now();
  while (completed_count < num_files &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(60)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_EQ(completed_count, num_files) << "Not all uploads completed";

  uploader.stop();
}


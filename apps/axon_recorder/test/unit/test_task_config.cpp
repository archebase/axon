/**
 * Unit tests for TaskConfig
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "../task_config.hpp"

using namespace axon::recorder;

// ============================================================================
// TaskConfig Tests
// ============================================================================

class TaskConfigTest : public ::testing::Test {
protected:
  TaskConfig create_sample_config() {
    TaskConfig config;
    config.task_id = "task_001";
    config.device_id = "robot_01";
    config.data_collector_id = "collector_01";
    config.order_id = "order_batch_001";
    config.operator_name = "john.doe";
    config.scene = "warehouse";
    config.subscene = "picking";
    config.skills = {"navigation", "manipulation"};
    config.factory = "factory_shanghai_01";
    config.topics = {"/camera/image", "/lidar/scan"};
    config.start_callback_url = "http://server/start";
    config.finish_callback_url = "http://server/finish";
    config.user_token = "jwt_token_123";
    return config;
  }
};

TEST_F(TaskConfigTest, DefaultConstruction) {
  TaskConfig config;

  EXPECT_TRUE(config.task_id.empty());
  EXPECT_TRUE(config.device_id.empty());
  EXPECT_TRUE(config.topics.empty());
}

TEST_F(TaskConfigTest, IsValid) {
  TaskConfig config;

  // Empty task_id is invalid
  EXPECT_FALSE(config.is_valid());

  // With task_id it's valid
  config.task_id = "test_task";
  EXPECT_TRUE(config.is_valid());
}

TEST_F(TaskConfigTest, HasCallbacks) {
  TaskConfig config;

  // No callbacks configured
  EXPECT_FALSE(config.has_callbacks());

  // Only start callback
  config.start_callback_url = "http://server/start";
  EXPECT_TRUE(config.has_callbacks());

  // Clear and only set finish callback
  config.start_callback_url.clear();
  config.finish_callback_url = "http://server/finish";
  EXPECT_TRUE(config.has_callbacks());

  // Both callbacks
  config.start_callback_url = "http://server/start";
  EXPECT_TRUE(config.has_callbacks());
}

TEST_F(TaskConfigTest, GenerateOutputFilename) {
  TaskConfig config;
  config.task_id = "task_123";

  EXPECT_EQ(config.generate_output_filename(), "task_123.mcap");
}

TEST_F(TaskConfigTest, GenerateOutputFilenameWithSpecialChars) {
  TaskConfig config;
  config.task_id = "task-abc_123";

  EXPECT_EQ(config.generate_output_filename(), "task-abc_123.mcap");
}

TEST_F(TaskConfigTest, NewFieldsOrderIdAndOperatorName) {
  TaskConfig config = create_sample_config();

  // Verify new fields are stored correctly
  EXPECT_EQ(config.order_id, "order_batch_001");
  EXPECT_EQ(config.operator_name, "john.doe");
}

TEST_F(TaskConfigTest, FactoryField) {
  TaskConfig config = create_sample_config();

  // Verify factory field (renamed from organization)
  EXPECT_EQ(config.factory, "factory_shanghai_01");
}

// ============================================================================
// TaskConfigCache Tests
// ============================================================================

class TaskConfigCacheTest : public ::testing::Test {
protected:
  TaskConfig create_sample_config(const std::string& task_id = "task_001") {
    TaskConfig config;
    config.task_id = task_id;
    config.device_id = "robot_01";
    config.scene = "warehouse";
    return config;
  }
};

TEST_F(TaskConfigCacheTest, InitialState) {
  TaskConfigCache cache;

  EXPECT_FALSE(cache.has_config());
  EXPECT_FALSE(cache.get().has_value());
  EXPECT_TRUE(cache.get_task_id().empty());
}

TEST_F(TaskConfigCacheTest, CacheAndRetrieve) {
  TaskConfigCache cache;
  auto config = create_sample_config();

  cache.cache(config);

  EXPECT_TRUE(cache.has_config());
  EXPECT_TRUE(cache.get().has_value());
  EXPECT_EQ(cache.get_task_id(), "task_001");

  auto retrieved = cache.get();
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->task_id, "task_001");
  EXPECT_EQ(retrieved->device_id, "robot_01");
  EXPECT_EQ(retrieved->scene, "warehouse");
}

TEST_F(TaskConfigCacheTest, CacheOverwritesPrevious) {
  TaskConfigCache cache;

  // Cache first config
  cache.cache(create_sample_config("task_001"));
  EXPECT_EQ(cache.get_task_id(), "task_001");

  // Cache second config - should overwrite
  cache.cache(create_sample_config("task_002"));
  EXPECT_EQ(cache.get_task_id(), "task_002");
  EXPECT_TRUE(cache.has_config());
}

TEST_F(TaskConfigCacheTest, Clear) {
  TaskConfigCache cache;
  cache.cache(create_sample_config());

  EXPECT_TRUE(cache.has_config());

  cache.clear();

  EXPECT_FALSE(cache.has_config());
  EXPECT_FALSE(cache.get().has_value());
  EXPECT_TRUE(cache.get_task_id().empty());
}

TEST_F(TaskConfigCacheTest, MatchesTaskId) {
  TaskConfigCache cache;

  // No config cached
  EXPECT_FALSE(cache.matches_task_id("task_001"));

  cache.cache(create_sample_config("task_001"));

  // Matching task_id
  EXPECT_TRUE(cache.matches_task_id("task_001"));

  // Non-matching task_id
  EXPECT_FALSE(cache.matches_task_id("task_002"));
  EXPECT_FALSE(cache.matches_task_id(""));
}

TEST_F(TaskConfigCacheTest, CachedAtTimestamp) {
  TaskConfigCache cache;
  auto before = std::chrono::system_clock::now();

  cache.cache(create_sample_config());

  auto after = std::chrono::system_clock::now();
  auto config = cache.get();

  ASSERT_TRUE(config.has_value());
  EXPECT_GE(config->cached_at, before);
  EXPECT_LE(config->cached_at, after);
}

TEST_F(TaskConfigCacheTest, ThreadSafety) {
  TaskConfigCache cache;
  const int num_threads = 4;
  const int iterations = 1000;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  // Writer threads
  for (int t = 0; t < num_threads / 2; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < iterations; ++i) {
        auto config = create_sample_config("task_" + std::to_string(t) + "_" + std::to_string(i));
        cache.cache(config);
        success_count++;
      }
    });
  }

  // Reader threads
  for (int t = 0; t < num_threads / 2; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < iterations; ++i) {
        auto config = cache.get();
        cache.has_config();
        cache.get_task_id();
        success_count++;
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  // All operations should complete without crash
  EXPECT_EQ(success_count.load(), num_threads * iterations);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

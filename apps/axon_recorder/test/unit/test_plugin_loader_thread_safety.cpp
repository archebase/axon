/**
 * Unit tests for PluginLoader thread safety
 * Tests for fix: Added std::mutex plugins_mutex_ to protect plugins_ map
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "../plugin_loader.hpp"

using namespace axon;

// ============================================================================
// PluginLoader Thread Safety Tests
// ============================================================================

class PluginLoaderThreadSafetyTest : public ::testing::Test {
protected:
  void SetUp() override {
    // PluginLoader starts empty
  }

  PluginLoader plugin_loader_;
};

// =============================================================================
// Test: Concurrent load() calls should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentLoadCallsShouldNotCrash) {
  // Note: This test uses the mock plugin which should be available
  // In a real scenario, we'd need to ensure the mock plugin exists
  constexpr int kNumThreads = 4;
  constexpr int kIterations = 10;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};
  std::atomic<int> failure_count{0};

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kIterations; ++i) {
        // Try to load a non-existent plugin (should fail gracefully)
        auto result = plugin_loader_.load("/nonexistent/plugin_" + std::to_string(t) + ".so");
        if (result.has_value()) {
          success_count++;
        } else {
          failure_count++;
        }
      }
    });
  }

  // Wait for all threads
  for (auto& thread : threads) {
    thread.join();
  }

  // All operations should complete without crash
  EXPECT_EQ(success_count.load() + failure_count.load(), kNumThreads * kIterations);

  // get_last_error() should be thread-safe
  std::string error = plugin_loader_.get_last_error();
  EXPECT_FALSE(error.empty());
}

// =============================================================================
// Test: Concurrent loaded_plugins() calls should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentLoadedPluginsCallsShouldNotCrash) {
  constexpr int kNumThreads = 8;
  constexpr int kIterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> call_count{0};

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < kIterations; ++i) {
        // Concurrent reads from loaded_plugins()
        auto plugins = plugin_loader_.loaded_plugins();
        call_count++;

        // Should return empty vector (no plugins loaded)
        EXPECT_TRUE(plugins.empty());
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(call_count.load(), kNumThreads * kIterations);
}

// =============================================================================
// Test: Concurrent load() and loaded_plugins() should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentLoadAndReadShouldNotCrash) {
  constexpr int kNumReaderThreads = 4;
  constexpr int kNumWriterThreads = 2;
  constexpr int kIterations = 50;

  std::vector<std::thread> threads;
  std::atomic<int> reader_count{0};
  std::atomic<int> writer_count{0};
  std::atomic<bool> running{true};

  // Reader threads - call loaded_plugins()
  for (int t = 0; t < kNumReaderThreads; ++t) {
    threads.emplace_back([&]() {
      while (running.load()) {
        auto plugins = plugin_loader_.loaded_plugins();
        reader_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  // Writer threads - try to load plugins
  for (int t = 0; t < kNumWriterThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kIterations; ++i) {
        auto result = plugin_loader_.load("/nonexistent/plugin_" + std::to_string(t) + ".so");
        writer_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });
  }

  // Wait for writer threads to finish
  for (int t = kNumReaderThreads; t < kNumReaderThreads + kNumWriterThreads; ++t) {
    threads[t].join();
  }

  // Stop reader threads
  running.store(false);
  for (int t = 0; t < kNumReaderThreads; ++t) {
    threads[t].join();
  }

  // All operations should complete without crash
  EXPECT_GT(reader_count.load(), 0);
  EXPECT_EQ(writer_count.load(), kNumWriterThreads * kIterations);
}

// =============================================================================
// Test: Concurrent is_loaded() calls should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentIsLoadedCallsShouldNotCrash) {
  constexpr int kNumThreads = 6;
  constexpr int kIterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> call_count{0};

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kIterations; ++i) {
        // Concurrent is_loaded() calls
        bool loaded = plugin_loader_.is_loaded("plugin_" + std::to_string(t));
        call_count++;

        // Should return false (no plugins loaded)
        EXPECT_FALSE(loaded);
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(call_count.load(), kNumThreads * kIterations);
}

// =============================================================================
// Test: Concurrent get_descriptor() calls should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentGetDescriptorCallsShouldNotCrash) {
  constexpr int kNumThreads = 6;
  constexpr int kIterations = 100;

  std::vector<std::thread> threads;
  std::atomic<int> call_count{0};

  for (int t = 0; t < kNumThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kIterations; ++i) {
        // Concurrent get_descriptor() calls
        const auto* desc = plugin_loader_.get_descriptor("plugin_" + std::to_string(t));
        call_count++;

        // Should return nullptr (plugin not loaded)
        EXPECT_EQ(desc, nullptr);
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  EXPECT_EQ(call_count.load(), kNumThreads * kIterations);
}

// =============================================================================
// Test: Concurrent unload_all() and loaded_plugins() should not crash
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, ConcurrentUnloadAndReadShouldNotCrash) {
  constexpr int kNumReaderThreads = 4;
  constexpr int kNumUnloaderThreads = 2;
  constexpr int kDurationMs = 100;

  std::vector<std::thread> threads;
  std::atomic<int> reader_count{0};
  std::atomic<int> unloader_count{0};
  std::atomic<bool> running{true};

  // Reader threads - call loaded_plugins()
  for (int t = 0; t < kNumReaderThreads; ++t) {
    threads.emplace_back([&]() {
      while (running.load()) {
        auto plugins = plugin_loader_.loaded_plugins();
        reader_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    });
  }

  // Unloader threads - call unload_all()
  for (int t = 0; t < kNumUnloaderThreads; ++t) {
    threads.emplace_back([&]() {
      while (running.load()) {
        plugin_loader_.unload_all();
        unloader_count++;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  // Run for a fixed duration
  std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));

  // Stop all threads
  running.store(false);
  for (auto& thread : threads) {
    thread.join();
  }

  // All operations should complete without crash
  EXPECT_GT(reader_count.load(), 0);
  EXPECT_GT(unloader_count.load(), 0);
}

// =============================================================================
// Test: Stress test with mixed operations
// =============================================================================
TEST_F(PluginLoaderThreadSafetyTest, StressTestMixedOperations) {
  constexpr int kTotalThreads = 10;
  constexpr int kDurationMs = 200;

  std::vector<std::thread> threads;
  std::atomic<bool> running{true};
  std::atomic<int> total_operations{0};

  // Mix of different operations
  for (int t = 0; t < kTotalThreads; ++t) {
    threads.emplace_back([&, t]() {
      int operation = t % 5;  // Different operation per thread

      while (running.load()) {
        switch (operation) {
          case 0: {
            auto plugins = plugin_loader_.loaded_plugins();
            total_operations++;
            break;
          }
          case 1: {
            bool loaded = plugin_loader_.is_loaded("test_plugin");
            total_operations++;
            break;
          }
          case 2: {
            const auto* desc = plugin_loader_.get_descriptor("test_plugin");
            total_operations++;
            break;
          }
          case 3: {
            auto result = plugin_loader_.load("/nonexistent/test.so");
            total_operations++;
            break;
          }
          case 4: {
            plugin_loader_.unload_all();
            total_operations++;
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  // Run for a fixed duration
  std::this_thread::sleep_for(std::chrono::milliseconds(kDurationMs));

  // Stop all threads
  running.store(false);
  for (auto& thread : threads) {
    thread.join();
  }

  // All operations should complete without crash
  EXPECT_GT(total_operations.load(), 0);
}

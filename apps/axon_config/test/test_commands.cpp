// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "commands.hpp"

namespace fs = std::filesystem;

namespace axon {
namespace config {
namespace test {

class CommandsTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Use a temporary test directory
    std::string dir_name = "axon_commands_test_" + std::to_string(std::time(nullptr));
    test_dir_ = fs::temp_directory_path() / dir_name;
    fs::create_directory(test_dir_);

    // Set config dir to test directory by setting environment
    // For testing, we use the default path and create it
    default_config_dir_ = fs::temp_directory_path();
    default_config_dir_ /= "axon_config_" + std::to_string(std::time(nullptr));
    fs::create_directory(default_config_dir_);
  }

  void TearDown() override {
    // Clean up test directories
    if (fs::exists(test_dir_)) {
      fs::remove_all(test_dir_);
    }
    if (fs::exists(default_config_dir_)) {
      fs::remove_all(default_config_dir_);
    }
  }

  // Helper to create a test file
  void create_test_file(
    const fs::path& dir, const std::string& rel_path, const std::string& content
  ) {
    fs::path file_path = dir;
    file_path /= rel_path;
    fs::create_directories(file_path.parent_path());

    std::ofstream file(file_path);
    file << content;
    file.close();
  }

  fs::path test_dir_;
  fs::path default_config_dir_;
};

// =============================================================================
// Test: execute command parsing - basic smoke tests
// =============================================================================

TEST(CommandsExecuteTest, HandlesHelpCommand) {
  Commands commands;
  char argv0[] = "axon_config";
  char* argv[] = {argv0};
  int result = commands.execute(1, argv);

  EXPECT_EQ(result, 0);  // help returns 0
}

TEST(CommandsExecuteTest, HandlesUnknownCommand) {
  Commands commands;
  char argv0[] = "axon_config";
  char argv1[] = "unknown";
  char* argv[] = {argv0, argv1};
  int result = commands.execute(2, argv);

  EXPECT_EQ(result, 1);  // unknown command returns error
}

}  // namespace test
}  // namespace config
}  // namespace axon

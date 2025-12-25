/**
 * Unit tests for MCAP Validator
 */

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "mcap_validator.hpp"
#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;
using namespace axon::mcap_wrapper;

class McapValidatorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create unique temp directory for each test
    test_dir_ = "/tmp/mcap_validator_test_" + 
                std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    fs::create_directories(test_dir_);
  }

  void TearDown() override {
    // Clean up test directory
    std::error_code ec;
    fs::remove_all(test_dir_, ec);
  }

  // Create a truly valid MCAP file using the McapWriterWrapper
  // This ensures the file passes full MCAP library validation
  std::string createValidMcapFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    
    axon::mcap_wrapper::McapWriterWrapper writer;
    bool opened = writer.open(path);
    EXPECT_TRUE(opened) << "Failed to open MCAP writer for: " << path;
    
    // Register schema first, then channel
    uint16_t schema_id = writer.register_schema("test_schema", "ros2msg", "string data");
    uint16_t channel_id = writer.register_channel("test_topic", "cdr", schema_id);
    
    // Write a simple message with correct signature:
    // write(channel_id, log_time_ns, publish_time_ns, data, data_size)
    std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
    writer.write(channel_id, 1000000000, 1000000000, data.data(), data.size());
    
    writer.close();
    return path;
  }
  
  // Create a minimal valid MCAP file (just header/footer, no data)
  std::string createMinimalValidMcapFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    
    axon::mcap_wrapper::McapWriterWrapper writer;
    bool opened = writer.open(path);
    EXPECT_TRUE(opened) << "Failed to open MCAP writer for: " << path;
    writer.close();
    
    return path;
  }
  
  // Create a file with correct magic bytes but invalid internal structure
  // (for testing header/footer validation separately)
  std::string createMagicOnlyFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    std::ofstream file(path, std::ios::binary);
    
    // MCAP magic bytes
    static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
    
    // Write header magic
    file.write(MAGIC, 8);
    
    // Write some padding (invalid records, but has correct magic)
    std::vector<char> padding(100, 0);
    file.write(padding.data(), padding.size());
    
    // Write footer magic
    file.write(MAGIC, 8);
    
    file.close();
    return path;
  }

  // Create file with invalid header magic
  std::string createInvalidHeaderFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    std::ofstream file(path, std::ios::binary);
    
    // Write wrong magic
    const char wrong_magic[] = {'\x00', 'X', 'X', 'X', 'X', '0', '\r', '\n'};
    file.write(wrong_magic, 8);
    
    // Write some padding
    std::vector<char> padding(100, 0);
    file.write(padding.data(), padding.size());
    
    // Write valid footer magic
    static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
    file.write(MAGIC, 8);
    
    file.close();
    return path;
  }

  // Create file with invalid footer magic (truncated)
  std::string createTruncatedFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    std::ofstream file(path, std::ios::binary);
    
    // Write valid header magic
    static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
    file.write(MAGIC, 8);
    
    // Write some data
    std::vector<char> data(100, 0);
    file.write(data.data(), data.size());
    
    // Don't write footer magic - simulates truncated file
    
    file.close();
    return path;
  }

  // Create empty file
  std::string createEmptyFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    std::ofstream file(path, std::ios::binary);
    file.close();
    return path;
  }

  // Create file that's too small
  std::string createTooSmallFile(const std::string& name) {
    std::string path = test_dir_ + "/" + name;
    std::ofstream file(path, std::ios::binary);
    
    // Write only 10 bytes (less than minimum 16 bytes)
    const char data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
    file.write(data, sizeof(data));
    
    file.close();
    return path;
  }

  std::string test_dir_;
};

// =============================================================================
// Header Validation Tests
// =============================================================================

TEST_F(McapValidatorTest, ValidMcapHeader) {
  // Use real MCAP file created by writer
  std::string path = createValidMcapFile("valid_header.mcap");
  
  auto result = validateMcapHeader(path);
  EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
  EXPECT_TRUE(result.error_message.empty());
}

TEST_F(McapValidatorTest, ValidMcapHeaderMagicOnly) {
  // Test with file that only has correct magic bytes (invalid structure)
  std::string path = createMagicOnlyFile("magic_only.mcap");
  
  auto result = validateMcapHeader(path);
  EXPECT_TRUE(result.valid) << "Header magic should be valid";
}

TEST_F(McapValidatorTest, InvalidMcapHeader) {
  std::string path = createInvalidHeaderFile("invalid_header.mcap");
  
  auto result = validateMcapHeader(path);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_NE(result.error_message.find("magic"), std::string::npos);
}

TEST_F(McapValidatorTest, HeaderFileNotFound) {
  auto result = validateMcapHeader("/nonexistent/file.mcap");
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
}

TEST_F(McapValidatorTest, HeaderFileTooSmall) {
  std::string path = createEmptyFile("empty.mcap");
  
  auto result = validateMcapHeader(path);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

TEST_F(McapValidatorTest, HeaderPartialRead) {
  std::string path = createTooSmallFile("partial.mcap");
  
  auto result = validateMcapHeader(path);
  EXPECT_FALSE(result.valid);
  // File is 10 bytes, can read 8 byte header but magic won't match
  EXPECT_FALSE(result.error_message.empty());
}

// =============================================================================
// Footer Validation Tests
// =============================================================================

TEST_F(McapValidatorTest, ValidMcapFooter) {
  // Use real MCAP file created by writer
  std::string path = createValidMcapFile("valid_footer.mcap");
  
  auto result = validateMcapFooter(path);
  EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
  EXPECT_TRUE(result.error_message.empty());
}

TEST_F(McapValidatorTest, ValidMcapFooterMagicOnly) {
  // Test with file that only has correct magic bytes (invalid structure)
  std::string path = createMagicOnlyFile("magic_only_footer.mcap");
  
  auto result = validateMcapFooter(path);
  EXPECT_TRUE(result.valid) << "Footer magic should be valid";
}

TEST_F(McapValidatorTest, InvalidMcapFooter) {
  std::string path = createTruncatedFile("truncated.mcap");
  
  auto result = validateMcapFooter(path);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, FooterFileNotFound) {
  auto result = validateMcapFooter("/nonexistent/file.mcap");
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
}

TEST_F(McapValidatorTest, FooterFileTooSmall) {
  std::string path = createTooSmallFile("too_small.mcap");
  
  auto result = validateMcapFooter(path);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

// =============================================================================
// Structure Validation Tests
// =============================================================================

TEST_F(McapValidatorTest, ValidMcapStructure) {
  // Use McapWriterWrapper to create a truly valid MCAP file
  std::string path = createValidMcapFile("valid_structure.mcap");
  
  auto result = validateMcapStructure(path);
  EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
  EXPECT_TRUE(result.error_message.empty());
}

TEST_F(McapValidatorTest, MinimalValidMcapStructure) {
  // Empty but valid MCAP (no channels, no messages)
  std::string path = createMinimalValidMcapFile("minimal.mcap");
  
  auto result = validateMcapStructure(path);
  EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
}

TEST_F(McapValidatorTest, StructureInvalidHeader) {
  std::string path = createInvalidHeaderFile("bad_header.mcap");
  
  auto result = validateMcapStructure(path);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, StructureInvalidFooter) {
  std::string path = createTruncatedFile("bad_footer.mcap");
  
  auto result = validateMcapStructure(path);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, StructureFileNotFound) {
  auto result = validateMcapStructure("/nonexistent/file.mcap");
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
}

// =============================================================================
// Convenience Function Tests
// =============================================================================

TEST_F(McapValidatorTest, IsValidMcapTrue) {
  // Use McapWriterWrapper to create a truly valid MCAP file
  std::string path = createValidMcapFile("valid.mcap");
  
  EXPECT_TRUE(isValidMcap(path)) << "Valid MCAP file should pass validation";
}

TEST_F(McapValidatorTest, IsValidMcapFalse) {
  std::string path = createInvalidHeaderFile("invalid.mcap");
  EXPECT_FALSE(isValidMcap(path));
}

TEST_F(McapValidatorTest, IsValidMcapNonexistent) {
  EXPECT_FALSE(isValidMcap("/nonexistent/file.mcap"));
}

// =============================================================================
// Result Object Tests
// =============================================================================

TEST_F(McapValidatorTest, ResultSuccess) {
  auto result = McapValidationResult::success();
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(static_cast<bool>(result));
}

TEST_F(McapValidatorTest, ResultFailure) {
  auto result = McapValidationResult::failure("Test error message");
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.error_message, "Test error message");
  EXPECT_FALSE(static_cast<bool>(result));
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F(McapValidatorTest, EmptyPath) {
  auto result = validateMcapStructure("");
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, DirectoryPath) {
  auto result = validateMcapStructure(test_dir_);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, SymbolicLink) {
  // Use McapWriterWrapper to create a truly valid MCAP file
  std::string valid_file = createValidMcapFile("original.mcap");
  std::string link_path = test_dir_ + "/link.mcap";
  
  // Create symbolic link
  std::error_code ec;
  fs::create_symlink(valid_file, link_path, ec);
  
  if (!ec) {
    // Full structure validation should work through symlinks
    auto result = validateMcapStructure(link_path);
    EXPECT_TRUE(result.valid) << "Symbolic links should be followed: " << result.error_message;
  }
}

// =============================================================================
// Header Validation Edge Cases
// =============================================================================

TEST_F(McapValidatorTest, HeaderCannotOpenFile) {
  // Create a file, then make it unreadable (if possible)
  std::string path = test_dir_ + "/unreadable.mcap";
  std::ofstream file(path, std::ios::binary);
  file << "test";
  file.close();
  
  // Remove read permission (Unix-specific)
  std::error_code ec;
  fs::permissions(path, fs::perms::none, ec);
  
  if (!ec) {
    auto result = validateMcapHeader(path);
    // Should fail to open file
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.error_message.empty());
    
    // Restore permissions for cleanup
    fs::permissions(path, fs::perms::owner_all, ec);
  }
}

// =============================================================================
// Footer Validation Edge Cases
// =============================================================================

TEST_F(McapValidatorTest, FooterCannotOpenFile) {
  std::string path = test_dir_ + "/unreadable_footer.mcap";
  std::ofstream file(path, std::ios::binary);
  // Write valid header magic + enough data
  static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  file.write(MAGIC, 8);
  std::vector<char> padding(100, 0);
  file.write(padding.data(), padding.size());
  file.write(MAGIC, 8);
  file.close();
  
  // Remove read permission
  std::error_code ec;
  fs::permissions(path, fs::perms::none, ec);
  
  if (!ec) {
    auto result = validateMcapFooter(path);
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.error_message.empty());
    
    // Restore permissions for cleanup
    fs::permissions(path, fs::perms::owner_all, ec);
  }
}

TEST_F(McapValidatorTest, FooterInvalidMagicBytes) {
  // Create file with valid header but invalid footer magic
  std::string path = test_dir_ + "/bad_footer_magic.mcap";
  std::ofstream file(path, std::ios::binary);
  
  // Write valid header magic
  static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  file.write(MAGIC, 8);
  
  // Write some padding
  std::vector<char> padding(100, 0);
  file.write(padding.data(), padding.size());
  
  // Write invalid footer magic
  static const char BAD_MAGIC[] = {'\x00', 'X', 'X', 'X', 'X', '0', '\r', '\n'};
  file.write(BAD_MAGIC, 8);
  
  file.close();
  
  auto result = validateMcapFooter(path);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("truncated"), std::string::npos);
}

// =============================================================================
// Structure Validation Edge Cases
// =============================================================================

TEST_F(McapValidatorTest, StructureWithMagicOnlyFile) {
  // File has correct header/footer magic but invalid internal structure
  std::string path = createMagicOnlyFile("structure_magic_only.mcap");
  
  // Full structure validation should fail (invalid summary)
  auto result = validateMcapStructure(path);
  EXPECT_FALSE(result.valid);
  // Should fail at the MCAP reader open or summary read step
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(McapValidatorTest, StructureValidationLogsErrors) {
  // Test files that trigger error logging paths in validateMcapStructure
  
  // Invalid header - should log header validation failure
  std::string bad_header_path = createInvalidHeaderFile("log_test_bad_header.mcap");
  auto result1 = validateMcapStructure(bad_header_path);
  EXPECT_FALSE(result1.valid);
  
  // Truncated file - should log footer validation failure
  std::string truncated_path = createTruncatedFile("log_test_truncated.mcap");
  auto result2 = validateMcapStructure(truncated_path);
  EXPECT_FALSE(result2.valid);
}

// =============================================================================
// Validation with Different File Sizes
// =============================================================================

TEST_F(McapValidatorTest, FileExactlyMinimumSize) {
  // Create a file that's exactly the minimum size (16 bytes = 2 * magic)
  std::string path = test_dir_ + "/exactly_min_size.mcap";
  std::ofstream file(path, std::ios::binary);
  
  static const char MAGIC[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  file.write(MAGIC, 8);  // Header magic
  file.write(MAGIC, 8);  // Footer magic (immediately after header)
  
  file.close();
  
  // Header validation should pass
  auto header_result = validateMcapHeader(path);
  EXPECT_TRUE(header_result.valid);
  
  // Footer validation should pass
  auto footer_result = validateMcapFooter(path);
  EXPECT_TRUE(footer_result.valid);
  
  // But full structure validation may fail (no valid records)
  auto structure_result = validateMcapStructure(path);
  // The MCAP library may or may not accept this
  // Just verify it doesn't crash
}

TEST_F(McapValidatorTest, FileJustBelowMinimumSize) {
  std::string path = test_dir_ + "/below_min_size.mcap";
  std::ofstream file(path, std::ios::binary);
  
  // Write only 15 bytes (just below minimum)
  static const char data[] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n', 
                               '\x89', 'M', 'C', 'A', 'P', '0', '\r'};
  file.write(data, 15);
  
  file.close();
  
  // Footer validation should fail (too small)
  auto result = validateMcapFooter(path);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

// =============================================================================
// Multiple Validations on Same File
// =============================================================================

TEST_F(McapValidatorTest, MultipleValidationsOnSameFile) {
  std::string path = createValidMcapFile("multi_validation.mcap");
  
  // Run validations multiple times to ensure no state leakage
  for (int i = 0; i < 3; ++i) {
    auto header_result = validateMcapHeader(path);
    EXPECT_TRUE(header_result.valid) << "Header validation failed on iteration " << i;
    
    auto footer_result = validateMcapFooter(path);
    EXPECT_TRUE(footer_result.valid) << "Footer validation failed on iteration " << i;
    
    auto structure_result = validateMcapStructure(path);
    EXPECT_TRUE(structure_result.valid) << "Structure validation failed on iteration " << i;
  }
}

// =============================================================================
// Large Valid MCAP File
// =============================================================================

TEST_F(McapValidatorTest, LargeValidMcapFile) {
  std::string path = test_dir_ + "/large_valid.mcap";
  
  axon::mcap_wrapper::McapWriterWrapper writer;
  ASSERT_TRUE(writer.open(path));
  
  uint16_t schema_id = writer.register_schema("large_test", "ros2msg", "uint8[] data");
  uint16_t channel_id = writer.register_channel("/large_topic", "cdr", schema_id);
  
  // Write many messages
  std::vector<uint8_t> data(1024, 0xAB);  // 1KB per message
  for (int i = 0; i < 1000; ++i) {
    uint64_t timestamp = 1000000000ULL + i * 1000000ULL;
    writer.write(channel_id, timestamp, timestamp, data.data(), data.size());
  }
  
  writer.close();
  
  // Validate the large file
  auto result = validateMcapStructure(path);
  EXPECT_TRUE(result.valid) << "Error: " << result.error_message;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

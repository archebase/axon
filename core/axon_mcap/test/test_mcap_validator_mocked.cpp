// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for MCAP Validator using GoogleMock
 *
 * These tests use mocks to test error paths that are difficult to simulate
 * with real files, such as I/O errors, permission errors, and MCAP reader failures.
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mcap/errors.hpp>
#include <mcap/types.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <string>

#include "mcap_validator.hpp"
#include "mcap_validator_impl.hpp"
#include "mcap_validator_mocks.hpp"
#include "mcap_validator_test_helpers.hpp"

namespace fs = std::filesystem;
using namespace axon::mcap_wrapper;
using namespace axon::mcap_wrapper::test;
using ::testing::_;
using ::testing::ByMove;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnRef;

// MCAP magic bytes for test data
static constexpr std::array<char, 8> MCAP_MAGIC = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};

class McapValidatorMockedTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_filesystem_ = std::make_unique<MockFileSystem>();
    mock_stream_factory_ = std::make_unique<MockFileStreamFactory>();
    mock_reader_ = std::make_unique<MockMcapReader>();
  }

  std::unique_ptr<MockFileSystem> mock_filesystem_;
  std::unique_ptr<MockFileStreamFactory> mock_stream_factory_;
  std::unique_ptr<MockMcapReader> mock_reader_;
  std::string test_path_ = "/test/path/file.mcap";
};

// =============================================================================
// I/O Error Tests - Header Validation
// =============================================================================

TEST_F(McapValidatorMockedTest, HeaderReadFailure) {
  // Test file read failure during header validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        // Fill buffer with magic bytes (even though read will fail)
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  EXPECT_CALL(*mock_stream, good())
    .WillOnce(Return(false));  // Read failed (short-circuits, gcount() not called)

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, HeaderPartialRead) {
  // Test partial read (less than MCAP_MAGIC_SIZE bytes)
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        // Fill buffer with partial magic bytes
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(4)));
      },
      ReturnRef(*mock_stream)
    ));
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(4));  // Only 4 bytes read (need 8)

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, HeaderCannotOpenFile) {
  // Test file cannot be opened
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::unique_ptr<IFileStream>()))
    );  // Factory returns nullptr (open failed)

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("Cannot open"), std::string::npos);
  EXPECT_NE(
    result.error_message.find(test_path_), std::string::npos
  );  // Verify path is in error message
}

// =============================================================================
// I/O Error Tests - Footer Validation
// =============================================================================

TEST_F(McapValidatorMockedTest, FooterSeekFailure) {
  // Test seek failure during footer validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(false));  // Seek failed

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("seek"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterReadFailure) {
  // Test read failure during footer validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, good())
    .WillOnce(Return(true))    // Seek succeeded
    .WillOnce(Return(false));  // Read failed
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  // good() returns false, so gcount() is not called due to short-circuit evaluation

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("read"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterPartialRead) {
  // Test partial read during footer validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, good())
    .WillOnce(Return(true))   // Seek succeeded
    .WillOnce(Return(true));  // Read succeeded (but partial)
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(4)));
      },
      ReturnRef(*mock_stream)
    ));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(4));  // Only 4 bytes read (need 8)

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("read"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterCannotOpenFile) {
  // Test file cannot be opened during footer validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));  // Large enough
  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::unique_ptr<IFileStream>()))
    );  // Factory returns nullptr (open failed)

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("Cannot open"), std::string::npos);
}

// =============================================================================
// Filesystem Error Tests
// =============================================================================

TEST_F(McapValidatorMockedTest, HeaderFileNotFound) {
  // Test file not found error
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(false));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
  EXPECT_NE(
    result.error_message.find(test_path_), std::string::npos
  );  // Verify path is in error message
}

TEST_F(McapValidatorMockedTest, FooterFileNotFound) {
  // Test file not found error - covers branch 1 on line 100 (!filesystem.exists(path) == true)
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(false));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
  EXPECT_NE(
    result.error_message.find(test_path_), std::string::npos
  );  // Verify path is in error message
}

TEST_F(McapValidatorMockedTest, FooterFileTooSmall) {
  // Test file too small for footer validation
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_))
    .WillOnce(Return(10));  // Less than MIN_MCAP_SIZE (16)

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

// =============================================================================
// MCAP Reader Error Tests
// =============================================================================

TEST_F(McapValidatorMockedTest, StructureReaderOpenFailure) {
  // Test MCAP reader open failure
  EXPECT_CALL(*mock_filesystem_, exists(test_path_))
    .WillOnce(Return(true))   // Header check
    .WillOnce(Return(true));  // Footer check
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  // Mock successful header validation
  auto mock_header_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_header_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_header_stream)
    ));
  EXPECT_CALL(*mock_header_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_header_stream, gcount()).WillOnce(Return(8));

  // Mock successful footer validation
  auto mock_footer_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_footer_stream, seekg(_, std::ios::end))
    .WillOnce(ReturnRef(*mock_footer_stream));
  EXPECT_CALL(*mock_footer_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_footer_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_footer_stream)
    ));
  EXPECT_CALL(*mock_footer_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_header_stream))))
    .WillOnce(Return(ByMove(std::move(mock_footer_stream))));

  // Mock reader open failure
  mcap::Status open_error = mcap::Status(mcap::StatusCode::OpenFailed, "Cannot open file");
  EXPECT_CALL(*mock_reader_, open(test_path_)).WillOnce(Return(open_error));

  auto result =
    validateMcapStructureImpl(test_path_, *mock_filesystem_, *mock_stream_factory_, *mock_reader_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("Cannot open MCAP"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, StructureReaderSummaryReadFailure) {
  // Test MCAP reader summary read failure
  EXPECT_CALL(*mock_filesystem_, exists(test_path_))
    .WillOnce(Return(true))   // Header check
    .WillOnce(Return(true));  // Footer check
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  // Mock successful header validation
  auto mock_header_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_header_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_header_stream)
    ));
  EXPECT_CALL(*mock_header_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_header_stream, gcount()).WillOnce(Return(8));

  // Mock successful footer validation
  auto mock_footer_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_footer_stream, seekg(_, std::ios::end))
    .WillOnce(ReturnRef(*mock_footer_stream));
  EXPECT_CALL(*mock_footer_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_footer_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_footer_stream)
    ));
  EXPECT_CALL(*mock_footer_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_header_stream))))
    .WillOnce(Return(ByMove(std::move(mock_footer_stream))));

  // Mock reader open success, but summary read failure
  mcap::Status open_success = mcap::Status(mcap::StatusCode::Success);
  mcap::Status summary_error =
    mcap::Status(mcap::StatusCode::InvalidFile, "Summary section invalid");
  EXPECT_CALL(*mock_reader_, open(test_path_)).WillOnce(Return(open_success));
  EXPECT_CALL(*mock_reader_, readSummary(_)).WillOnce(Return(summary_error));
  EXPECT_CALL(*mock_reader_, close());

  auto result =
    validateMcapStructureImpl(test_path_, *mock_filesystem_, *mock_stream_factory_, *mock_reader_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("summary section"), std::string::npos);
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST_F(McapValidatorMockedTest, HeaderInvalidMagicBytes) {
  // Test invalid magic bytes in header
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));

  auto mock_stream = std::make_unique<MockFileStream>();
  std::array<char, 8> invalid_magic = {'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [&invalid_magic](char* buffer, std::streamsize) {
        std::memcpy(buffer, invalid_magic.data(), 8);
      },
      ReturnRef(*mock_stream)
    ));
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("magic"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterInvalidMagicBytes) {
  // Test invalid magic bytes in footer
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, good()).WillRepeatedly(Return(true));
  std::array<char, 8> invalid_magic = {'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [&invalid_magic](char* buffer, std::streamsize) {
        std::memcpy(buffer, invalid_magic.data(), 8);
      },
      ReturnRef(*mock_stream)
    ));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("truncated"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, StructureHeaderValidationFailure) {
  // Test that structure validation fails early on header failure
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(false));

  // Should not call footer validation or reader if header fails
  EXPECT_CALL(*mock_filesystem_, file_size(_)).Times(0);
  EXPECT_CALL(*mock_reader_, open(_)).Times(0);

  auto result =
    validateMcapStructureImpl(test_path_, *mock_filesystem_, *mock_stream_factory_, *mock_reader_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, StructureFooterValidationFailure) {
  // Test that structure validation fails on footer failure (after header passes)
  EXPECT_CALL(*mock_filesystem_, exists(test_path_))
    .WillOnce(Return(true))    // Header check
    .WillOnce(Return(false));  // Footer check (file deleted between checks)

  // Mock successful header validation
  auto mock_header_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_header_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_header_stream)
    ));
  EXPECT_CALL(*mock_header_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_header_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_header_stream))));

  // Should not call reader if footer fails
  EXPECT_CALL(*mock_reader_, open(_)).Times(0);

  auto result =
    validateMcapStructureImpl(test_path_, *mock_filesystem_, *mock_stream_factory_, *mock_reader_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("not exist"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, StructureCompleteSuccessPath) {
  // Test the complete success path through validateMcapStructureImpl
  // This ensures line 186 (AXON_LOG_DEBUG for success) is covered
  EXPECT_CALL(*mock_filesystem_, exists(test_path_))
    .WillOnce(Return(true))   // Header check
    .WillOnce(Return(true));  // Footer check
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  // Mock successful header validation
  auto mock_header_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_header_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_header_stream)
    ));
  EXPECT_CALL(*mock_header_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_header_stream, gcount()).WillOnce(Return(8));

  // Mock successful footer validation
  auto mock_footer_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_footer_stream, seekg(_, std::ios::end))
    .WillOnce(ReturnRef(*mock_footer_stream));
  EXPECT_CALL(*mock_footer_stream, good()).WillRepeatedly(Return(true));
  EXPECT_CALL(*mock_footer_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_footer_stream)
    ));
  EXPECT_CALL(*mock_footer_stream, gcount()).WillOnce(Return(8));

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_header_stream))))
    .WillOnce(Return(ByMove(std::move(mock_footer_stream))));

  // Mock successful reader operations
  mcap::Status open_success = mcap::Status(mcap::StatusCode::Success);
  mcap::Status summary_success = mcap::Status(mcap::StatusCode::Success);
  EXPECT_CALL(*mock_reader_, open(test_path_)).WillOnce(Return(open_success));
  EXPECT_CALL(*mock_reader_, readSummary(_)).WillOnce(Return(summary_success));
  EXPECT_CALL(*mock_reader_, close());

  auto result =
    validateMcapStructureImpl(test_path_, *mock_filesystem_, *mock_stream_factory_, *mock_reader_);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.error_message.empty());
}

TEST_F(McapValidatorMockedTest, HeaderGoodButWrongGcount) {
  // Test case where good() returns true but gcount() != MCAP_MAGIC_SIZE
  // This tests the second part of the OR condition on line 75
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        // Fill with magic bytes
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  // good() returns true, but gcount() returns wrong value (simulates unusual I/O condition)
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(7));  // Only 7 bytes read, need 8

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterGoodButWrongGcount) {
  // Test case where good() returns true but gcount() != MCAP_MAGIC_SIZE for footer
  // This tests the second part of the OR condition on line 126
  // Using gcount() = 7 (less than MCAP_MAGIC_SIZE) to cover branch 3
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  // good() returns true for both seek and read, but gcount() returns wrong value
  EXPECT_CALL(*mock_stream, good())
    .WillOnce(Return(true))                                 // Seek succeeded
    .WillOnce(Return(true));                                // Read succeeded (but partial)
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(7));  // Only 7 bytes read, need 8

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("read"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, HeaderGoodButGcountGreaterThanMagicSize) {
  // Test case to cover branch 5 on line 75: good() == true && gcount() > MCAP_MAGIC_SIZE
  // This tests a different branch combination than gcount() < MCAP_MAGIC_SIZE
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        // Fill with magic bytes
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  // good() returns true, but gcount() returns value greater than MCAP_MAGIC_SIZE
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(9));  // 9 bytes read, need 8

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapHeaderImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("too small"), std::string::npos);
}

TEST_F(McapValidatorMockedTest, FooterGoodButGcountGreaterThanMagicSize) {
  // Test case to cover branch 5 on line 126: good() == true && gcount() > MCAP_MAGIC_SIZE
  EXPECT_CALL(*mock_filesystem_, exists(test_path_)).WillOnce(Return(true));
  EXPECT_CALL(*mock_filesystem_, file_size(test_path_)).WillOnce(Return(1000));

  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, seekg(_, std::ios::end)).WillOnce(ReturnRef(*mock_stream));
  EXPECT_CALL(*mock_stream, read(_, _))
    .WillOnce(DoAll(
      [](char* buffer, std::streamsize size) {
        std::memcpy(buffer, MCAP_MAGIC.data(), std::min(size, static_cast<std::streamsize>(8)));
      },
      ReturnRef(*mock_stream)
    ));
  // good() returns true for both seek and read, but gcount() returns value greater than
  // MCAP_MAGIC_SIZE
  EXPECT_CALL(*mock_stream, good())
    .WillOnce(Return(true))                                 // Seek succeeded
    .WillOnce(Return(true));                                // Read succeeded
  EXPECT_CALL(*mock_stream, gcount()).WillOnce(Return(9));  // 9 bytes read, need 8

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_path_, std::ios::binary))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  auto result = validateMcapFooterImpl(test_path_, *mock_filesystem_, *mock_stream_factory_);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error_message.find("read"), std::string::npos);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

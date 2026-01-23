// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

/**
 * Unit tests for S3Client using GoogleMock
 *
 * These tests use mocks to test error paths in file I/O operations
 * that are difficult to simulate with real files.
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "s3_client_test_helpers.hpp"
#include "uploader_mocks.hpp"

using namespace axon::uploader;
using namespace axon::uploader::test;
using ::testing::_;
using ::testing::ByMove;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnRef;

class S3ClientMockedTest : public ::testing::Test {
protected:
  void SetUp() override {
    mock_stream_factory_ = std::make_unique<MockFileStreamFactory>();
  }

  std::unique_ptr<MockFileStreamFactory> mock_stream_factory_;
  std::string test_file_path_ = "/test/path/file.mcap";
};

// =============================================================================
// File Size Retrieval Error Tests
// =============================================================================

TEST_F(S3ClientMockedTest, GetFileSizeForUpload_StreamFactoryReturnsNullptr) {
  // Test file size retrieval when stream factory returns nullptr (file open failure)
  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_file_path_, _))
    .WillOnce(Return(ByMove(std::unique_ptr<IFileStream>())));

  uint64_t size = getFileSizeForUploadImpl(test_file_path_, *mock_stream_factory_);
  EXPECT_EQ(size, 0);  // Indicates failure
}

TEST_F(S3ClientMockedTest, GetFileSizeForUpload_StreamGoodButTellgFails) {
  // Test file size retrieval when tellg() returns error indicator
  // Implementation should detect tellg() error and return 0
  // Note: Due to short-circuit evaluation, if tellg() returns -1, good() is never called
  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, tellg()).WillOnce(Return(std::streampos(-1)));  // Error indicator
  // good() is NOT called due to short-circuit evaluation in: (pos == -1 || !file->good())

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_file_path_, _))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  uint64_t size = getFileSizeForUploadImpl(test_file_path_, *mock_stream_factory_);
  // tellg() returns -1 on error, which should be detected and return 0 (failure indicator)
  EXPECT_EQ(size, 0);
}

TEST_F(S3ClientMockedTest, GetFileSizeForUpload_Success) {
  // Test successful file size retrieval
  // Implementation calls tellg() and then checks good() to verify stream state
  constexpr uint64_t expected_size = 2048;
  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, tellg()).WillOnce(Return(std::streampos(expected_size)));
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));  // Stream is in good state

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_file_path_, _))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  uint64_t size = getFileSizeForUploadImpl(test_file_path_, *mock_stream_factory_);
  EXPECT_EQ(size, expected_size);
}

TEST_F(S3ClientMockedTest, GetFileSizeForUpload_EmptyFile) {
  // Test file size retrieval for empty file (size = 0)
  // Implementation calls tellg() and then checks good() to verify stream state
  auto mock_stream = std::make_unique<MockFileStream>();
  EXPECT_CALL(*mock_stream, tellg()).WillOnce(Return(std::streampos(0)));
  EXPECT_CALL(*mock_stream, good()).WillOnce(Return(true));  // Stream is in good state

  EXPECT_CALL(*mock_stream_factory_, create_file_stream(test_file_path_, _))
    .WillOnce(Return(ByMove(std::move(mock_stream))));

  uint64_t size = getFileSizeForUploadImpl(test_file_path_, *mock_stream_factory_);
  EXPECT_EQ(size, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

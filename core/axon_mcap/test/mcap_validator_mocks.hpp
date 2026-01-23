// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_MCAP_VALIDATOR_MOCKS_HPP
#define AXON_MCAP_VALIDATOR_MOCKS_HPP

#include <gmock/gmock.h>
#include <mcap/errors.hpp>
#include <mcap/reader.hpp>

#include <ios>
#include <memory>
#include <string>

#include "mcap_validator_interfaces.hpp"

namespace axon {
namespace mcap_wrapper {
namespace test {

/**
 * Mock implementation of IFileSystem for testing
 */
class MockFileSystem : public IFileSystem {
public:
  MOCK_METHOD(bool, exists, (const std::string& path), (const, override));
  MOCK_METHOD(uint64_t, file_size, (const std::string& path), (const, override));
};

/**
 * Mock implementation of IFileStream for testing
 */
class MockFileStream : public IFileStream {
public:
  MOCK_METHOD(IFileStream&, read, (char* buffer, std::streamsize size), (override));
  MOCK_METHOD(
    IFileStream&, seekg, (std::streamoff offset, std::ios_base::seekdir origin), (override)
  );
  MOCK_METHOD(std::streamsize, gcount, (), (const, override));
  MOCK_METHOD(bool, good, (), (const, override));
  MOCK_METHOD(bool, fail, (), (const, override));
  MOCK_METHOD(bool, bad, (), (const, override));
};

/**
 * Mock implementation of IFileStreamFactory for testing
 */
class MockFileStreamFactory : public IFileStreamFactory {
public:
  MOCK_METHOD(
    std::unique_ptr<IFileStream>, create_file_stream,
    (const std::string& path, std::ios_base::openmode mode), (override)
  );
};

/**
 * Mock implementation of IMcapReader for testing
 */
class MockMcapReader : public IMcapReader {
public:
  MOCK_METHOD(mcap::Status, open, (const std::string& path), (override));
  MOCK_METHOD(mcap::Status, readSummary, (mcap::ReadSummaryMethod method), (override));
  MOCK_METHOD(void, close, (), (override));
};

/**
 * Helper to create a mock file stream that can be returned from factory
 */
class MockFileStreamHelper {
public:
  static std::unique_ptr<MockFileStream> create() {
    return std::make_unique<MockFileStream>();
  }
};

}  // namespace test
}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_VALIDATOR_MOCKS_HPP

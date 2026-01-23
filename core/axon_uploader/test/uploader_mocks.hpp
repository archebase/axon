// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_UPLOADER_MOCKS_HPP
#define AXON_UPLOADER_MOCKS_HPP

#include <gmock/gmock.h>

#include <ios>
#include <memory>
#include <string>

#include "uploader_interfaces.hpp"

namespace axon {
namespace uploader {
namespace test {

/**
 * Mock implementation of IFileSystem for testing
 */
class MockFileSystem : public IFileSystem {
public:
  MOCK_METHOD(bool, exists, (const std::string& path), (const, override));
  MOCK_METHOD(uint64_t, file_size, (const std::string& path), (const, override));
  MOCK_METHOD(bool, remove, (const std::string& path), (const, override));
  MOCK_METHOD(
    bool, rename, (const std::string& old_path, const std::string& new_path), (const, override)
  );
  MOCK_METHOD(bool, create_directories, (const std::string& path), (const, override));
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
  MOCK_METHOD(std::streampos, tellg, (), (override));
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

}  // namespace test
}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOADER_MOCKS_HPP

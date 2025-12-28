#ifndef AXON_UPLOADER_IMPL_HPP
#define AXON_UPLOADER_IMPL_HPP

#include <filesystem>
#include <fstream>
#include <memory>

#include "uploader_interfaces.hpp"

namespace axon {
namespace uploader {

/**
 * Default implementation of IFileSystem using std::filesystem
 */
class FileSystemImpl : public IFileSystem {
public:
  bool exists(const std::string& path) const override {
    return std::filesystem::exists(path); // LCOV_EXCL_BR_LINE
  }

  uint64_t file_size(const std::string& path) const override {
    return static_cast<uint64_t>(std::filesystem::file_size(path)); // LCOV_EXCL_BR_LINE
  }

  bool remove(const std::string& path) const override {
    std::error_code ec;
    return std::filesystem::remove(path, ec); // LCOV_EXCL_BR_LINE
  }

  bool rename(const std::string& old_path, const std::string& new_path) const override {
    std::error_code ec;
    std::filesystem::rename(old_path, new_path, ec); // LCOV_EXCL_BR_LINE
    return !ec;
  }

  bool create_directories(const std::string& path) const override {
    std::error_code ec;
    return std::filesystem::create_directories(path, ec); // LCOV_EXCL_BR_LINE
  }
};

/**
 * Default implementation of IFileStream using std::ifstream
 */
class FileStreamImpl : public IFileStream {
public:
  explicit FileStreamImpl(const std::string& path, std::ios_base::openmode mode)
      : stream_(path, mode) {} // LCOV_EXCL_BR_LINE

  IFileStream& read(char* buffer, std::streamsize size) override {
    stream_.read(buffer, size);
    return *this;
  }

  IFileStream& seekg(std::streamoff offset, std::ios_base::seekdir origin) override {
    stream_.seekg(offset, origin);
    return *this;
  }

  std::streampos tellg() override {
    return stream_.tellg();
  }

  std::streamsize gcount() const override {
    return stream_.gcount();
  }

  bool good() const override {
    return stream_.good();
  }

  bool fail() const override {
    return stream_.fail();
  }

  bool bad() const override {
    return stream_.bad();
  }

private:
  std::ifstream stream_;
};

/**
 * Default implementation of IFileStreamFactory
 */
class FileStreamFactoryImpl : public IFileStreamFactory {
public:
  std::unique_ptr<IFileStream> create_file_stream(
    const std::string& path, std::ios_base::openmode mode
  ) override {
    // LCOV_EXCL_BR_START - Smart pointer operations generate exception-safety branches
    // that are standard library implementation details
    auto stream = std::make_unique<FileStreamImpl>(path, mode);
    if (!stream->good()) {
      return nullptr;
    }
    return stream;
    // LCOV_EXCL_BR_STOP
  }
};

}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOADER_IMPL_HPP

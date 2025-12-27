#ifndef AXON_MCAP_VALIDATOR_IMPL_HPP
#define AXON_MCAP_VALIDATOR_IMPL_HPP

#include "mcap_validator_interfaces.hpp"

#include <filesystem>
#include <fstream>
#include <memory>

#include <mcap/reader.hpp>
#include <mcap/errors.hpp>

namespace axon {
namespace mcap_wrapper {

/**
 * Default implementation of IFileSystem using std::filesystem
 */
class FileSystemImpl : public IFileSystem {
public:
  bool exists(const std::string& path) const override {
    return std::filesystem::exists(path);
  }

  uint64_t file_size(const std::string& path) const override {
    return static_cast<uint64_t>(std::filesystem::file_size(path));
  }
};

/**
 * Default implementation of IFileStream using std::ifstream
 */
class FileStreamImpl : public IFileStream {
public:
  explicit FileStreamImpl(const std::string& path, std::ios_base::openmode mode)
      : stream_(path, mode) {}

  IFileStream& read(char* buffer, std::streamsize size) override {
    stream_.read(buffer, size);
    return *this;
  }

  IFileStream& seekg(std::streamoff offset, std::ios_base::seekdir origin) override {
    stream_.seekg(offset, origin);
    return *this;
  }

  std::streamsize gcount() const override { return stream_.gcount(); }

  bool good() const override { return stream_.good(); }

  bool fail() const override { return stream_.fail(); }

  bool bad() const override { return stream_.bad(); }

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
    auto stream = std::make_unique<FileStreamImpl>(path, mode);
    if (!stream->good()) {
      return nullptr;
    }
    return stream;
  }
};

/**
 * Default implementation of IMcapReader using mcap::McapReader
 */
class McapReaderImpl : public IMcapReader {
public:
  mcap::Status open(const std::string& path) override { return reader_.open(path); }

  mcap::Status readSummary(mcap::ReadSummaryMethod method) override {
    return reader_.readSummary(method);
  }

  void close() override { reader_.close(); }

private:
  mcap::McapReader reader_;
};

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_VALIDATOR_IMPL_HPP


#ifndef AXON_MCAP_VALIDATOR_INTERFACES_HPP
#define AXON_MCAP_VALIDATOR_INTERFACES_HPP

#include <cstddef>
#include <cstdint>
#include <ios>
#include <memory>
#include <string>

// Forward declarations for MCAP types
namespace mcap {
class Status;
enum class ReadSummaryMethod;
}  // namespace mcap

namespace axon {
namespace mcap_wrapper {

/**
 * Interface for filesystem operations
 * Allows mocking filesystem operations for testing
 */
class IFileSystem {
public:
  virtual ~IFileSystem() = default;

  /**
   * Check if a file or directory exists
   */
  virtual bool exists(const std::string& path) const = 0;

  /**
   * Get the size of a file in bytes
   */
  virtual uint64_t file_size(const std::string& path) const = 0;
};

/**
 * Interface for file stream operations
 * Allows mocking file I/O operations for testing
 */
class IFileStream {
public:
  virtual ~IFileStream() = default;

  /**
   * Read data from the file stream
   * @param buffer Buffer to read into
   * @param size Number of bytes to read
   * @return Reference to this stream for chaining
   */
  virtual IFileStream& read(char* buffer, std::streamsize size) = 0;

  /**
   * Seek to a position in the file
   * @param offset Offset from origin
   * @param origin Seek origin (beg, cur, end)
   * @return Reference to this stream for chaining
   */
  virtual IFileStream& seekg(std::streamoff offset, std::ios_base::seekdir origin) = 0;

  /**
   * Get the number of characters read in the last read operation
   */
  virtual std::streamsize gcount() const = 0;

  /**
   * Check if the stream is in a good state
   */
  virtual bool good() const = 0;

  /**
   * Check if the stream is in a failed state
   */
  virtual bool fail() const = 0;

  /**
   * Check if the stream is in a bad state
   */
  virtual bool bad() const = 0;
};

/**
 * Interface for MCAP reader operations
 * Allows mocking MCAP library operations for testing
 */
class IMcapReader {
public:
  virtual ~IMcapReader() = default;

  /**
   * Open an MCAP file for reading
   * @param path Path to the MCAP file
   * @return Status object indicating success or failure
   */
  virtual mcap::Status open(const std::string& path) = 0;

  /**
   * Read the MCAP summary section
   * @param method Read method (NoFallbackScan, etc.)
   * @return Status object indicating success or failure
   */
  virtual mcap::Status readSummary(mcap::ReadSummaryMethod method) = 0;

  /**
   * Close the MCAP file
   */
  virtual void close() = 0;
};

/**
 * Factory interface for creating file streams
 * Allows mocking file stream creation for testing
 */
class IFileStreamFactory {
public:
  virtual ~IFileStreamFactory() = default;

  /**
   * Create a file stream for reading
   * @param path Path to the file
   * @param mode File open mode
   * @return Unique pointer to the file stream, or nullptr on failure
   */
  virtual std::unique_ptr<IFileStream> create_file_stream(
    const std::string& path, std::ios_base::openmode mode
  ) = 0;
};

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_VALIDATOR_INTERFACES_HPP

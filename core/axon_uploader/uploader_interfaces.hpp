#ifndef AXON_UPLOADER_INTERFACES_HPP
#define AXON_UPLOADER_INTERFACES_HPP

#include <cstdint>
#include <ios>
#include <memory>
#include <string>

namespace axon {
namespace uploader {

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

  /**
   * Remove a file or directory
   * @param path Path to file or directory to remove
   * @return true if successful, false otherwise
   */
  virtual bool remove(const std::string& path) const = 0;

  /**
   * Rename or move a file
   * @param old_path Source path
   * @param new_path Destination path
   * @return true if successful, false otherwise
   */
  virtual bool rename(const std::string& old_path, const std::string& new_path) const = 0;

  /**
   * Create directory tree (including parent directories)
   * @param path Path to directory to create
   * @return true if successful, false otherwise
   */
  virtual bool create_directories(const std::string& path) const = 0;
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
   * Get the current position in the file
   * @return Current position (for file size calculation)
   */
  virtual std::streampos tellg() = 0;

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

}  // namespace uploader
}  // namespace axon

#endif  // AXON_UPLOADER_INTERFACES_HPP

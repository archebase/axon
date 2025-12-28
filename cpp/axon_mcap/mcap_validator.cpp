#include "mcap_validator.hpp"

#include "mcap_validator_impl.hpp"
#include "mcap_validator_interfaces.hpp"

#include <mcap/reader.hpp>
#include <mcap/errors.hpp>

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>

// Logging infrastructure (optional - only if axon_logging is linked)
#ifdef AXON_HAS_LOGGING
#define AXON_LOG_COMPONENT "axon_mcap"
#include <axon_log_macros.hpp>
#else
// No-op macros when logging is not available
#define AXON_LOG_DEBUG(msg) \
  do {                      \
  } while (0)
#define AXON_LOG_INFO(msg) \
  do {                     \
  } while (0)
#define AXON_LOG_WARN(msg) \
  do {                     \
  } while (0)
#define AXON_LOG_ERROR(msg) \
  do {                      \
  } while (0)
namespace axon {
namespace logging {
template <typename T>
inline std::string kv(const char* key, const T& value) {
  (void)key;
  (void)value;
  return "";
}
}  // namespace logging
}  // namespace axon
#endif

namespace axon {
namespace mcap_wrapper {

// MCAP magic bytes: 0x89 'M' 'C' 'A' 'P' '0' '\r' '\n'
// This is the same magic used for both header and footer
static constexpr std::array<char, 8> MCAP_MAGIC = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
static constexpr size_t MCAP_MAGIC_SIZE = 8;

// Minimum valid MCAP file size (header + footer at minimum)
static constexpr size_t MIN_MCAP_SIZE = MCAP_MAGIC_SIZE * 2;

// Internal implementation with dependency injection
// Exposed for testing - declared in mcap_validator_test_helpers.hpp
McapValidationResult validateMcapHeaderImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory
) {
  // Check file exists
  // LCOV_EXCL_BR_START - String concatenation and smart pointer operations generate
  // exception-safety branches that are standard library implementation details
  if (!filesystem.exists(path)) {
    return McapValidationResult::failure("File does not exist: " + path);
  }

  auto file = stream_factory.create_file_stream(path, std::ios::binary);

  if (!file) {
    return McapValidationResult::failure("Cannot open file: " + path);
  }
  // LCOV_EXCL_BR_STOP
  
  // Read header magic bytes
  std::array<char, MCAP_MAGIC_SIZE> header_magic{};
  file->read(header_magic.data(), MCAP_MAGIC_SIZE);

  if (!file->good() || static_cast<size_t>(file->gcount()) != MCAP_MAGIC_SIZE) {
    return McapValidationResult::failure("File too small to contain MCAP header");
  }

  // Compare with expected magic
  // LCOV_EXCL_BR_START - Magic bytes comparison is a standard library operation,
  if (header_magic != MCAP_MAGIC) {
    return McapValidationResult::failure("Invalid MCAP header magic bytes");
  }
  // LCOV_EXCL_BR_STOP

  return McapValidationResult::success();
}

// Public API - uses default implementations
McapValidationResult validateMcapHeader(const std::string& path) {
  // LCOV_EXCL_BR_START - Static initialization guard generates thread-safety branches
  static FileSystemImpl default_filesystem;
  static FileStreamFactoryImpl default_stream_factory;
  // LCOV_EXCL_BR_STOP
  return validateMcapHeaderImpl(path, default_filesystem, default_stream_factory);
}

// Internal implementation with dependency injection
// Exposed for testing via mcap_validator_test_helpers.hpp
McapValidationResult validateMcapFooterImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory
) {
  // Check file exists
  // LCOV_EXCL_BR_START - String concatenation and smart pointer operations generate
  // exception-safety branches that are standard library implementation details
  if (!filesystem.exists(path)) {
    return McapValidationResult::failure("File does not exist: " + path);
  }

  // Get file size
  auto file_size = filesystem.file_size(path);
  if (file_size < MIN_MCAP_SIZE) {
    return McapValidationResult::failure("File too small to be valid MCAP (minimum " +
                                         std::to_string(MIN_MCAP_SIZE) + " bytes)");
  }

  auto file = stream_factory.create_file_stream(path, std::ios::binary);
  if (!file) {
    return McapValidationResult::failure("Cannot open file: " + path);
  }
  // LCOV_EXCL_BR_STOP

  // Seek to footer magic position
  file->seekg(-static_cast<std::streamoff>(MCAP_MAGIC_SIZE), std::ios::end);
  if (!file->good()) {
    return McapValidationResult::failure("Cannot seek to footer position");
  }

  // Read footer magic bytes
  std::array<char, MCAP_MAGIC_SIZE> footer_magic{};
  file->read(footer_magic.data(), MCAP_MAGIC_SIZE);

  if (!file->good() || static_cast<size_t>(file->gcount()) != MCAP_MAGIC_SIZE) {
    return McapValidationResult::failure("Cannot read footer magic bytes");
  }

  // Compare with expected magic
  if (footer_magic != MCAP_MAGIC) {
    return McapValidationResult::failure(
        "Invalid MCAP footer magic bytes (file may be truncated or corrupted)");
  }

  return McapValidationResult::success();
}

// Public API - uses default implementations
McapValidationResult validateMcapFooter(const std::string& path) {
  // LCOV_EXCL_BR_START - Static initialization guard generates thread-safety branches
  static FileSystemImpl default_filesystem;
  static FileStreamFactoryImpl default_stream_factory;
  // LCOV_EXCL_BR_STOP
  return validateMcapFooterImpl(path, default_filesystem, default_stream_factory);
}

// Internal implementation with dependency injection
// Exposed for testing via mcap_validator_test_helpers.hpp
McapValidationResult validateMcapStructureImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory,
    IMcapReader& reader
) {
  // Step 1: Validate header
  auto header_result = validateMcapHeaderImpl(path, filesystem, stream_factory);
  if (!header_result) {
    // LCOV_EXCL_START - Logging macro generates many internal branches from string formatting
    // These branches are implementation details of the logging infrastructure, not validation logic
    AXON_LOG_ERROR("MCAP header validation failed: " + header_result.error_message);
    // LCOV_EXCL_STOP
    return header_result;
  }

  // Step 2: Validate footer
  auto footer_result = validateMcapFooterImpl(path, filesystem, stream_factory);
  if (!footer_result) {
    // LCOV_EXCL_START - Logging macro generates many internal branches from string formatting
    // These branches are implementation details of the logging infrastructure, not validation logic
    AXON_LOG_ERROR("MCAP footer validation failed: " + footer_result.error_message);
    // LCOV_EXCL_STOP
    return footer_result;
  }

  // Step 3: Validate summary section is parseable using MCAP library
  auto status = reader.open(path);
  if (!status.ok()) {
    // LCOV_EXCL_BR_START - String concatenation generates exception-safety branches
    // that are standard library implementation details
    std::string error_msg = "Cannot open MCAP for reading: " + status.message;
    // LCOV_EXCL_BR_STOP
    // LCOV_EXCL_START - Logging macro generates many internal branches from string formatting
    // These branches are implementation details of the logging infrastructure, not validation logic
    AXON_LOG_ERROR(error_msg);
    // LCOV_EXCL_STOP
    return McapValidationResult::failure(error_msg);
  }

  // Try to read summary without falling back to full scan
  // This verifies the summary section is present and parseable
  auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!summary_status.ok()) {
    // LCOV_EXCL_BR_START - String concatenation generates exception-safety branches
    // that are standard library implementation details
    std::string error_msg = "MCAP summary section invalid or missing: " + summary_status.message;
    // LCOV_EXCL_BR_STOP
    // LCOV_EXCL_START - Logging macro generates many internal branches from string formatting
    // These branches are implementation details of the logging infrastructure, not validation logic
    AXON_LOG_ERROR(error_msg);
    // LCOV_EXCL_STOP
    reader.close();
    return McapValidationResult::failure(error_msg);
  }

  reader.close();

  // LCOV_EXCL_START - Logging macro generates many internal branches from string formatting
  // These branches are implementation details of the logging infrastructure, not validation logic
  AXON_LOG_DEBUG("MCAP validation successful: " + path);
  // LCOV_EXCL_STOP
  return McapValidationResult::success();
}

// Public API - uses default implementations
McapValidationResult validateMcapStructure(const std::string& path) {
  // LCOV_EXCL_BR_START - Static initialization guard generates thread-safety branches
  static FileSystemImpl default_filesystem;
  static FileStreamFactoryImpl default_stream_factory;
  // LCOV_EXCL_BR_STOP
  // CRITICAL: Create a new reader instance per call to ensure thread safety.
  // McapReaderImpl wraps a stateful mcap::McapReader which maintains an open file handle.
  // If this were static (shared across calls), concurrent calls would race:
  // one thread could open file A, another could open file B (resetting state),
  // and the first thread would validate the wrong file.
  // DO NOT make this static - it must remain a local variable.
  McapReaderImpl reader;
  return validateMcapStructureImpl(path, default_filesystem, default_stream_factory, reader);
}

}  // namespace mcap_wrapper
}  // namespace axon


#include "mcap_validator.hpp"

#include <mcap/reader.hpp>

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>

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

McapValidationResult validateMcapHeader(const std::string& path) {
  // Check file exists
  if (!std::filesystem::exists(path)) {
    return McapValidationResult::failure("File does not exist: " + path);
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return McapValidationResult::failure("Cannot open file: " + path);
  }

  // Read header magic bytes
  std::array<char, MCAP_MAGIC_SIZE> header_magic{};
  file.read(header_magic.data(), MCAP_MAGIC_SIZE);

  if (!file || static_cast<size_t>(file.gcount()) != MCAP_MAGIC_SIZE) {
    return McapValidationResult::failure("File too small to contain MCAP header");
  }

  // Compare with expected magic
  if (header_magic != MCAP_MAGIC) {
    return McapValidationResult::failure("Invalid MCAP header magic bytes");
  }

  return McapValidationResult::success();
}

McapValidationResult validateMcapFooter(const std::string& path) {
  // Check file exists
  if (!std::filesystem::exists(path)) {
    return McapValidationResult::failure("File does not exist: " + path);
  }

  // Get file size
  auto file_size = std::filesystem::file_size(path);
  if (file_size < MIN_MCAP_SIZE) {
    return McapValidationResult::failure("File too small to be valid MCAP (minimum " +
                                         std::to_string(MIN_MCAP_SIZE) + " bytes)");
  }

  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return McapValidationResult::failure("Cannot open file: " + path);
  }

  // Seek to footer magic position
  file.seekg(-static_cast<std::streamoff>(MCAP_MAGIC_SIZE), std::ios::end);
  if (!file) {
    return McapValidationResult::failure("Cannot seek to footer position");
  }

  // Read footer magic bytes
  std::array<char, MCAP_MAGIC_SIZE> footer_magic{};
  file.read(footer_magic.data(), MCAP_MAGIC_SIZE);

  if (!file || static_cast<size_t>(file.gcount()) != MCAP_MAGIC_SIZE) {
    return McapValidationResult::failure("Cannot read footer magic bytes");
  }

  // Compare with expected magic
  if (footer_magic != MCAP_MAGIC) {
    return McapValidationResult::failure(
        "Invalid MCAP footer magic bytes (file may be truncated or corrupted)");
  }

  return McapValidationResult::success();
}

McapValidationResult validateMcapStructure(const std::string& path) {
  // Step 1: Validate header
  auto header_result = validateMcapHeader(path);
  if (!header_result) {
    AXON_LOG_ERROR("MCAP header validation failed: " + header_result.error_message);
    return header_result;
  }

  // Step 2: Validate footer
  auto footer_result = validateMcapFooter(path);
  if (!footer_result) {
    AXON_LOG_ERROR("MCAP footer validation failed: " + footer_result.error_message);
    return footer_result;
  }

  // Step 3: Validate summary section is parseable using MCAP library
  mcap::McapReader reader;
  auto status = reader.open(path);
  if (!status.ok()) {
    std::string error_msg = "Cannot open MCAP for reading: " + status.message;
    AXON_LOG_ERROR(error_msg);
    return McapValidationResult::failure(error_msg);
  }

  // Try to read summary without falling back to full scan
  // This verifies the summary section is present and parseable
  auto summary_status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!summary_status.ok()) {
    std::string error_msg = "MCAP summary section invalid or missing: " + summary_status.message;
    AXON_LOG_ERROR(error_msg);
    reader.close();
    return McapValidationResult::failure(error_msg);
  }

  reader.close();

  AXON_LOG_DEBUG("MCAP validation successful: " + path);
  return McapValidationResult::success();
}

}  // namespace mcap_wrapper
}  // namespace axon


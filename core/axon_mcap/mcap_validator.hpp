#ifndef AXON_MCAP_VALIDATOR_HPP
#define AXON_MCAP_VALIDATOR_HPP

#include <string>

namespace axon {
namespace mcap_wrapper {

/**
 * Result of MCAP validation
 */
struct McapValidationResult {
  bool valid;
  std::string error_message;

  explicit operator bool() const {
    return valid;
  }

  static McapValidationResult success() {
    return {true, ""};
  }

  static McapValidationResult failure(const std::string& message) {
    return {false, message};
  }
};

/**
 * Validate MCAP file structural integrity
 *
 * This performs fast structural validation suitable for pre-upload checks:
 * - Header magic bytes verification
 * - Footer magic bytes verification
 * - Summary section parseability check
 *
 * It does NOT decode message payloads (that's the responsibility of
 * content QA at the edge layer).
 *
 * @param path Path to the MCAP file
 * @return McapValidationResult with success/failure and error message
 */
McapValidationResult validateMcapStructure(const std::string& path);

/**
 * Validate only the MCAP header magic bytes
 *
 * Fast check that verifies the file starts with valid MCAP magic bytes.
 * Useful for quick sanity checks.
 *
 * @param path Path to the MCAP file
 * @return McapValidationResult with success/failure and error message
 */
McapValidationResult validateMcapHeader(const std::string& path);

/**
 * Validate only the MCAP footer magic bytes
 *
 * Checks that the file ends with valid MCAP footer magic bytes.
 * Detects truncated files.
 *
 * @param path Path to the MCAP file
 * @return McapValidationResult with success/failure and error message
 */
McapValidationResult validateMcapFooter(const std::string& path);

/**
 * Convenience function that returns bool for simple validation
 *
 * @param path Path to the MCAP file
 * @return true if file is structurally valid, false otherwise
 */
inline bool isValidMcap(const std::string& path) {
  return validateMcapStructure(path).valid;
}

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_VALIDATOR_HPP

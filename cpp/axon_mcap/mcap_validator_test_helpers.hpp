#ifndef AXON_MCAP_VALIDATOR_TEST_HELPERS_HPP
#define AXON_MCAP_VALIDATOR_TEST_HELPERS_HPP

// This header is for testing only - exposes internal implementations
// for dependency injection in tests

#include "mcap_validator_interfaces.hpp"
#include "mcap_validator.hpp"

namespace axon {
namespace mcap_wrapper {

// Forward declarations of internal implementations
// These are defined in mcap_validator.cpp
McapValidationResult validateMcapHeaderImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory
);

McapValidationResult validateMcapFooterImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory
);

McapValidationResult validateMcapStructureImpl(
    const std::string& path, const IFileSystem& filesystem, IFileStreamFactory& stream_factory,
    IMcapReader& reader
);

}  // namespace mcap_wrapper
}  // namespace axon

#endif  // AXON_MCAP_VALIDATOR_TEST_HELPERS_HPP


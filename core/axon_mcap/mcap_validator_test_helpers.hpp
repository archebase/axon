// Copyright (c) 2026 ArcheBase
// Axon is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
//          http://license.coscl.org.cn/MulanPSL2
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
// See the Mulan PSL v2 for more details.

#ifndef AXON_MCAP_VALIDATOR_TEST_HELPERS_HPP
#define AXON_MCAP_VALIDATOR_TEST_HELPERS_HPP

// This header is for testing only - exposes internal implementations
// for dependency injection in tests

#include "mcap_validator.hpp"
#include "mcap_validator_interfaces.hpp"

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

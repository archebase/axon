// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_RPC_RESPONSE_HPP
#define AXON_SYSTEM_RPC_RESPONSE_HPP

#include <nlohmann/json.hpp>

#include <string>

namespace axon {
namespace system {

struct RpcResponse {
  bool success = true;
  std::string message;
  nlohmann::json data;

  nlohmann::json to_json() const {
    nlohmann::json result;
    result["success"] = success;
    result["message"] = message;
    if (!data.is_null()) {
      result["data"] = data;
    }
    return result;
  }
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_RPC_RESPONSE_HPP

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_RPC_RESPONSE_HPP
#define AXON_AGENT_RPC_RESPONSE_HPP

#include <nlohmann/json.hpp>

#include <string>

namespace axon {
namespace agent {

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

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_RPC_RESPONSE_HPP

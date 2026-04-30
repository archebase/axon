// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "websocket_rpc_handler.hpp"

#include <stdexcept>

// Logging infrastructure
#define AXON_LOG_COMPONENT "websocket_rpc_handler"
#include <axon_log_macros.hpp>

namespace axon {
namespace recorder {

WebSocketRpcHandler::WebSocketRpcHandler(const RpcCallbacks& callbacks, ResponseSender sender)
    : callbacks_(callbacks)
    , send_response_(std::move(sender)) {}

void WebSocketRpcHandler::handle_message(const std::string& client_id, const std::string& message) {
  try {
    nlohmann::json msg = nlohmann::json::parse(message);

    // Check if message has an action field
    if (!msg.contains("action")) {
      send_response(
        client_id, "", {false, "Missing 'action' field in request", nlohmann::json::object()}
      );
      return;
    }

    std::string action = msg["action"].get<std::string>();
    std::string request_id;

    // Extract request_id if present
    if (msg.contains("request_id")) {
      request_id = msg["request_id"].get<std::string>();
    }

    AXON_LOG_DEBUG(
      "WebSocket RPC from client " << client_id << ": action=" << action
                                   << ", request_id=" << request_id
    );

    // Route to appropriate handler
    // Pass the entire message as params (excluding 'action' and 'request_id')
    nlohmann::json params = msg;
    params.erase("action");
    params.erase("request_id");
    nlohmann::json response_json = route_action(action, params);

    // Add request_id to response if provided
    if (!request_id.empty()) {
      response_json["request_id"] = request_id;
    }

    // Set message type to rpc_response
    response_json["type"] = "rpc_response";

    // Send response to client
    send_response_(client_id, response_json);

  } catch (const nlohmann::json::exception& e) {
    AXON_LOG_ERROR("JSON parse error in WebSocket RPC: " << e.what());
    send_response(
      client_id, "", {false, std::string("JSON parse error: ") + e.what(), nlohmann::json::object()}
    );
  } catch (const std::exception& e) {
    AXON_LOG_ERROR("Error in WebSocket RPC handler: " << e.what());
    send_response(
      client_id, "", {false, std::string("Internal error: ") + e.what(), nlohmann::json::object()}
    );
  }
}

nlohmann::json WebSocketRpcHandler::route_action(
  const std::string& action, const nlohmann::json& params
) {
  // Map action strings to handler functions
  if (action == "begin") {
    return handle_rpc_begin(callbacks_, params).to_json();
  } else if (action == "finish") {
    return handle_rpc_finish(callbacks_, params).to_json();
  } else if (action == "pause") {
    return handle_rpc_pause(callbacks_, params).to_json();
  } else if (action == "resume") {
    return handle_rpc_resume(callbacks_, params).to_json();
  } else if (action == "cancel") {
    return handle_rpc_cancel(callbacks_, params).to_json();
  } else if (action == "clear") {
    return handle_rpc_clear(callbacks_, params).to_json();
  } else if (action == "config") {
    return handle_rpc_config(callbacks_, params).to_json();
  } else if (action == "quit") {
    return handle_rpc_quit(callbacks_, params).to_json();
  } else if (action == "get_state") {
    return handle_rpc_get_state(callbacks_, params).to_json();
  } else if (action == "get_stats") {
    return handle_rpc_get_stats(callbacks_, params).to_json();
  } else if (action == "get_drop_stats") {
    return handle_rpc_get_drop_stats(callbacks_, params).to_json();
  } else if (action == "get_latency_stats") {
    return handle_rpc_get_latency_stats(callbacks_, params).to_json();
  } else {
    // Unknown action
    nlohmann::json error_response;
    error_response["success"] = false;
    error_response["message"] = "Unknown RPC action: " + action;
    error_response["data"] = nlohmann::json::object();
    return error_response;
  }
}

nlohmann::json WebSocketRpcHandler::build_response(
  const std::string& request_id, const RpcResponse& response
) {
  nlohmann::json msg = response.to_json();
  msg["type"] = "rpc_response";
  if (!request_id.empty()) {
    msg["request_id"] = request_id;
  }
  return msg;
}

void WebSocketRpcHandler::send_response(
  const std::string& client_id, const std::string& request_id, const RpcResponse& response
) {
  nlohmann::json msg = build_response(request_id, response);
  send_response_(client_id, msg);
}

}  // namespace recorder
}  // namespace axon

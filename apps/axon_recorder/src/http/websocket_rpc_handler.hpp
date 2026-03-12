// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_WEBSOCKET_RPC_HANDLER_HPP
#define AXON_RECORDER_WEBSOCKET_RPC_HANDLER_HPP

#include <nlohmann/json.hpp>

#include <functional>
#include <string>

#include "rpc_handlers.hpp"

namespace axon {
namespace recorder {

/**
 * WebSocket RPC Handler
 *
 * Routes WebSocket messages to shared RPC handler functions.
 * Provides request-response correlation for concurrent requests.
 */
class WebSocketRpcHandler {
public:
  /**
   * Response sender callback type
   * Used to send RPC responses back to specific clients
   */
  using ResponseSender =
    std::function<void(const std::string& client_id, const nlohmann::json& response)>;

  /**
   * Constructor
   * @param callbacks RPC callbacks to invoke for commands
   * @param sender Function to send responses back to clients
   */
  WebSocketRpcHandler(const RpcCallbacks& callbacks, ResponseSender sender);

  /**
   * Handle incoming WebSocket message
   * @param client_id Unique identifier for the client
   * @param message JSON message from client
   */
  void handle_message(const std::string& client_id, const std::string& message);

private:
  /**
   * Route action string to appropriate handler function
   * @param action RPC action name
   * @param params Request parameters
   * @return RPC response as JSON
   */
  nlohmann::json route_action(const std::string& action, const nlohmann::json& params);

  /**
   * Build RPC response message
   * @param request_id Request ID for correlation
   * @param response RPC response
   * @return JSON message to send to client
   */
  nlohmann::json build_response(const std::string& request_id, const RpcResponse& response);

  /**
   * Send response to client
   * @param client_id Target client ID
   * @param request_id Request ID for correlation
   * @param response RPC response
   */
  void send_response(
    const std::string& client_id, const std::string& request_id, const RpcResponse& response
  );

  RpcCallbacks callbacks_;
  ResponseSender send_response_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_WEBSOCKET_RPC_HANDLER_HPP

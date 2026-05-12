// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_HTTP_SERVER_HPP
#define AXON_AGENT_HTTP_SERVER_HPP

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "agent_service.hpp"

namespace axon {
namespace agent {

class HttpServer {
public:
  HttpServer(std::string host, std::uint16_t port, AgentService& service);
  ~HttpServer();

  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;

  bool start();
  void stop();
  bool is_running() const;

private:
  using Request = boost::beast::http::request<boost::beast::http::string_body>;
  using Response = boost::beast::http::response<boost::beast::http::string_body>;

  void run();
  void handle_session(boost::asio::ip::tcp::socket socket);
  Response route_request(const Request& request);
  RpcResponse route_rpc(
    const std::string& path, const std::string& method, const nlohmann::json& params
  );
  static Response make_response(
    boost::beast::http::status status, const std::string& content_type, const std::string& body,
    unsigned int version, bool keep_alive
  );
  static std::string index_html();

  std::string host_;
  std::uint16_t port_;
  AgentService& service_;
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_{false};
};

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_HTTP_SERVER_HPP

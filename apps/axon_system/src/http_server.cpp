// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "http_server.hpp"

#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

namespace axon {
namespace system {

namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

HttpServer::HttpServer(std::string host, std::uint16_t port, SystemService& service)
    : host_(std::move(host))
    , port_(port)
    , service_(service) {}

HttpServer::~HttpServer() {
  stop();
}

bool HttpServer::start() {
  if (running_) {
    return true;
  }

  try {
    tcp::endpoint endpoint(boost::asio::ip::make_address(host_), port_);
    acceptor_ = std::make_unique<tcp::acceptor>(io_context_);
    acceptor_->open(endpoint.protocol());
    acceptor_->set_option(boost::asio::socket_base::reuse_address(true));
    acceptor_->bind(endpoint);
    acceptor_->listen(boost::asio::socket_base::max_listen_connections);
    acceptor_->non_blocking(true);
    running_ = true;
    thread_ = std::make_unique<std::thread>(&HttpServer::run, this);
    return true;
  } catch (const std::exception& ex) {
    std::cerr << "axon-system HTTP server failed to start: " << ex.what() << std::endl;
    running_ = false;
    return false;
  }
}

void HttpServer::stop() {
  if (!running_) {
    return;
  }

  running_ = false;
  boost::system::error_code ec;
  if (acceptor_) {
    acceptor_->close(ec);
  }
  io_context_.stop();
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}

bool HttpServer::is_running() const {
  return running_;
}

void HttpServer::run() {
  while (running_) {
    boost::system::error_code ec;
    tcp::socket socket(io_context_);
    acceptor_->accept(socket, ec);
    if (ec) {
      if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      if (running_) {
        std::cerr << "axon-system accept failed: " << ec.message() << std::endl;
      }
      continue;
    }
    std::thread(&HttpServer::handle_session, this, std::move(socket)).detach();
  }
}

void HttpServer::handle_session(tcp::socket socket) {
  beast::flat_buffer buffer;
  boost::system::error_code ec;
  Request request;
  http::read(socket, buffer, request, ec);
  if (ec) {
    return;
  }

  auto response = route_request(request);
  http::write(socket, response, ec);
  socket.shutdown(tcp::socket::shutdown_send, ec);
}

HttpServer::Response HttpServer::route_request(const Request& request) {
  const std::string target(request.target());
  const std::string method(request.method_string());

  if ((target == "/" || target == "/health") && method == "GET") {
    const auto rpc_response = service_.get_health();
    return make_response(
      rpc_response.success ? http::status::ok : http::status::service_unavailable,
      "application/json",
      rpc_response.to_json().dump(2),
      request.version(),
      request.keep_alive()
    );
  }

  if (target == "/rpc/state" && method == "GET") {
    const auto rpc_response = service_.get_state();
    return make_response(
      http::status::ok,
      "application/json",
      rpc_response.to_json().dump(2),
      request.version(),
      request.keep_alive()
    );
  }

  if (target == "/rpc/quit" && method == "POST") {
    const auto rpc_response = service_.request_shutdown();
    return make_response(
      http::status::ok,
      "application/json",
      rpc_response.to_json().dump(2),
      request.version(),
      request.keep_alive()
    );
  }

  return make_response(
    http::status::not_found,
    "application/json",
    nlohmann::json({{"success", false}, {"message", "not found"}}).dump(2),
    request.version(),
    request.keep_alive()
  );
}

HttpServer::Response HttpServer::make_response(
  http::status status, const std::string& content_type, const std::string& body,
  unsigned int version, bool keep_alive
) {
  Response response{status, version};
  response.set(http::field::server, "axon-system");
  response.set(http::field::content_type, content_type);
  response.keep_alive(keep_alive);
  response.body() = body;
  response.prepare_payload();
  return response;
}

}  // namespace system
}  // namespace axon

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "http_server.hpp"

#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <sys/time.h>

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

namespace {

void set_native_socket_timeout(tcp::socket& socket, int timeout_ms) {
  const timeval timeout{
    timeout_ms / 1000,
    static_cast<suseconds_t>((timeout_ms % 1000) * 1000),
  };
  setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket.native_handle(), SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

}  // namespace

struct HttpServer::Session {
  explicit Session(boost::asio::io_context& io_context)
      : stream(io_context) {}

  beast::tcp_stream stream;
  std::thread thread;
  std::atomic<bool> finished{false};
};

HttpServer::HttpServer(
  std::string host, std::uint16_t port, SystemService& service, int session_timeout_ms
)
    : host_(std::move(host))
    , port_(port)
    , service_(service)
    , session_timeout_ms_(session_timeout_ms > 0 ? session_timeout_ms : 5000) {}

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
    port_ = acceptor_->local_endpoint().port();
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
  if (!running_ && (!thread_ || !thread_->joinable())) {
    reap_sessions(true);
    return;
  }

  running_ = false;
  boost::system::error_code ec;
  if (acceptor_) {
    acceptor_->close(ec);
  }

  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& session : sessions_) {
      if (session && session->stream.socket().is_open()) {
        session->stream.socket().shutdown(tcp::socket::shutdown_both, ec);
        ec.clear();
        session->stream.socket().close(ec);
        ec.clear();
      }
    }
  }

  io_context_.stop();
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  reap_sessions(true);
}

bool HttpServer::is_running() const {
  return running_;
}

std::uint16_t HttpServer::port() const {
  return port_;
}

void HttpServer::run() {
  while (running_) {
    boost::system::error_code ec;
    auto session = std::make_shared<Session>(io_context_);
    acceptor_->accept(session->stream.socket(), ec);
    if (ec) {
      if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
        reap_sessions(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      if (running_) {
        std::cerr << "axon-system accept failed: " << ec.message() << std::endl;
      }
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(sessions_mutex_);
      sessions_.push_back(session);
    }

    try {
      session->thread = std::thread(&HttpServer::handle_session, this, session);
    } catch (const std::exception& ex) {
      session->finished = true;
      session->stream.socket().close(ec);
      std::cerr << "axon-system session failed to start: " << ex.what() << std::endl;
    }
    reap_sessions(false);
  }
}

void HttpServer::handle_session(std::shared_ptr<Session> session) {
  beast::flat_buffer buffer;
  boost::system::error_code ec;
  Request request;
  set_native_socket_timeout(session->stream.socket(), session_timeout_ms_);
  std::weak_ptr<Session> weak_session = session;
  std::thread([weak_session, timeout_ms = session_timeout_ms_]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
    const auto locked = weak_session.lock();
    if (!locked || locked->finished.load()) {
      return;
    }
    boost::system::error_code timeout_ec;
    if (locked->stream.socket().is_open()) {
      locked->stream.socket().shutdown(tcp::socket::shutdown_both, timeout_ec);
      timeout_ec.clear();
      locked->stream.socket().close(timeout_ec);
    }
  }).detach();
  session->stream.expires_after(std::chrono::milliseconds(session_timeout_ms_));
  http::read(session->stream, buffer, request, ec);
  if (ec) {
    session->stream.socket().close(ec);
    session->finished = true;
    return;
  }

  auto response = route_request(request);
  session->stream.expires_after(std::chrono::milliseconds(session_timeout_ms_));
  http::write(session->stream, response, ec);
  session->stream.socket().shutdown(tcp::socket::shutdown_send, ec);
  session->stream.socket().close(ec);
  session->finished = true;
}

void HttpServer::reap_sessions(bool join_all) {
  std::vector<std::shared_ptr<Session>> sessions_to_join;
  {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    auto it = sessions_.begin();
    while (it != sessions_.end()) {
      const bool finished = (*it)->finished.load();
      if (join_all || finished) {
        sessions_to_join.push_back(*it);
        it = sessions_.erase(it);
      } else {
        ++it;
      }
    }
  }

  const auto current_thread = std::this_thread::get_id();
  for (const auto& session : sessions_to_join) {
    if (session->thread.joinable() && session->thread.get_id() != current_thread) {
      session->thread.join();
    } else if (session->thread.joinable()) {
      session->thread.detach();
    }
  }
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

  if (target == "/rpc/metrics" && method == "GET") {
    const auto rpc_response = service_.get_metrics();
    return make_response(
      http::status::ok,
      "application/json",
      rpc_response.to_json().dump(2),
      request.version(),
      request.keep_alive()
    );
  }

  if (target == "/rpc/processes" && method == "GET") {
    const auto rpc_response = service_.get_processes();
    return make_response(
      http::status::ok,
      "application/json",
      rpc_response.to_json().dump(2),
      request.version(),
      request.keep_alive()
    );
  }

  if (target == "/rpc/alerts" && method == "GET") {
    const auto rpc_response = service_.get_alerts();
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

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_SYSTEM_HTTP_SERVER_HPP
#define AXON_SYSTEM_HTTP_SERVER_HPP

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "system_service.hpp"

namespace axon {
namespace system {

class HttpServer {
public:
  HttpServer(std::string host, std::uint16_t port, SystemService& service);
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
  static Response make_response(
    boost::beast::http::status status, const std::string& content_type, const std::string& body,
    unsigned int version, bool keep_alive
  );

  std::string host_;
  std::uint16_t port_;
  SystemService& service_;
  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::tcp::acceptor> acceptor_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_{false};
};

}  // namespace system
}  // namespace axon

#endif  // AXON_SYSTEM_HTTP_SERVER_HPP

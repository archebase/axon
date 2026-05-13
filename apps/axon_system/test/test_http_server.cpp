// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "http_server.hpp"

namespace {

namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

std::filesystem::path make_temp_dir(const std::string& name) {
  const auto base = std::filesystem::temp_directory_path() /
                    (name + "_" + std::to_string(getpid()) + "_" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(base);
  return base;
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

tcp::resolver::results_type resolve_loopback(
  boost::asio::io_context& io_context, std::uint16_t port
) {
  tcp::resolver resolver(io_context);
  return resolver.resolve("127.0.0.1", std::to_string(port));
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_http");
  try {
    axon::system::SystemService service(root);
    std::string error;
    require(service.initialize(&error), "initialize failed: " + error);

    axon::system::HttpServer server("127.0.0.1", 0, service, 100);
    require(server.start(), "server start failed");
    require(server.port() != 0, "server did not expose assigned port");

    {
      boost::asio::io_context io_context;
      beast::tcp_stream stream(io_context);
      stream.expires_after(std::chrono::seconds(1));
      stream.connect(resolve_loopback(io_context, server.port()));

      http::request<http::empty_body> request{http::verb::get, "/health", 11};
      request.set(http::field::host, "127.0.0.1");
      http::write(stream, request);

      beast::flat_buffer buffer;
      http::response<http::string_body> response;
      http::read(stream, buffer, response);
      require(response.result() == http::status::ok, "health status");
      require(response.body().find("\"success\": true") != std::string::npos, "health body");
    }

    {
      boost::asio::io_context io_context;
      beast::tcp_stream stream(io_context);
      stream.expires_after(std::chrono::seconds(1));
      stream.connect(resolve_loopback(io_context, server.port()));
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      std::array<char, 1> buffer{};
      boost::system::error_code ec;
      stream.socket().non_blocking(true, ec);
      require(!ec, "failed to set client non-blocking mode: " + ec.message());
      stream.socket().read_some(boost::asio::buffer(buffer), ec);
      require(ec.value() != 0, "slow connection should not produce response bytes");
      require(
        ec == boost::asio::error::eof || ec == boost::asio::error::would_block ||
          ec == boost::asio::error::connection_reset,
        "unexpected slow connection error: " + ec.message()
      );
      require(ec != boost::asio::error::would_block, "slow connection was not closed by timeout");
    }

    server.stop();
    service.mark_stopped();
    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}

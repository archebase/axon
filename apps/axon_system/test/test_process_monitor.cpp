// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>

#include "process_monitor.hpp"

namespace {

namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

struct HttpFixtureServer {
  HttpFixtureServer(std::uint16_t assigned_port, std::thread server_thread)
      : port(assigned_port)
      , thread(std::move(server_thread)) {}

  HttpFixtureServer(const HttpFixtureServer&) = delete;
  HttpFixtureServer& operator=(const HttpFixtureServer&) = delete;

  HttpFixtureServer(HttpFixtureServer&& other) noexcept
      : port(other.port)
      , thread(std::move(other.thread)) {}

  HttpFixtureServer& operator=(HttpFixtureServer&& other) noexcept {
    if (this != &other) {
      if (thread.joinable()) {
        thread.join();
      }
      port = other.port;
      thread = std::move(other.thread);
    }
    return *this;
  }

  ~HttpFixtureServer() {
    if (thread.joinable()) {
      thread.join();
    }
  }

  std::uint16_t port = 0;
  std::thread thread;
};

HttpFixtureServer start_http_response_server(
  const std::string& body, http::status status = http::status::ok
) {
  std::promise<std::uint16_t> port_promise;
  auto port_future = port_promise.get_future();

  std::thread server_thread([body, status, promise = std::move(port_promise)]() mutable {
    bool promise_satisfied = false;
    try {
      boost::asio::io_context io_context;
      tcp::acceptor acceptor(
        io_context, {boost::asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(0)}
      );
      promise.set_value(acceptor.local_endpoint().port());
      promise_satisfied = true;

      tcp::socket socket(io_context);
      acceptor.accept(socket);

      beast::flat_buffer buffer;
      http::request<http::string_body> request;
      boost::system::error_code ec;
      http::read(socket, buffer, request, ec);
      if (ec) {
        return;
      }

      http::response<http::string_body> response{status, request.version()};
      response.set(http::field::server, "axon-system-test");
      response.set(http::field::content_type, "application/json");
      response.body() = body;
      response.prepare_payload();
      http::write(socket, response, ec);
      socket.shutdown(tcp::socket::shutdown_both, ec);
    } catch (...) {
      if (!promise_satisfied) {
        promise.set_exception(std::current_exception());
      }
    }
  });

  return HttpFixtureServer(port_future.get(), std::move(server_thread));
}

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

void write_file(const std::filesystem::path& path, const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    throw std::runtime_error("failed to write " + path.string());
  }
  output << content;
}

std::string stat_line(
  int pid, const std::string& comm, std::uint64_t utime, std::uint64_t stime,
  std::uint64_t start_time
) {
  return std::to_string(pid) + " (" + comm + ") S 1 1 1 0 0 0 0 0 0 0 " + std::to_string(utime) +
         " " + std::to_string(stime) + " 0 0 20 0 1 0 " + std::to_string(start_time) +
         " 0 0 0 0 0 0 0 0 0\n";
}

void write_process(
  const std::filesystem::path& proc, int pid, const std::string& executable, std::uint64_t utime,
  std::uint64_t stime, std::uint64_t start_time, std::uint64_t rss_pages, std::uint64_t read_bytes,
  std::uint64_t write_bytes
) {
  const auto pid_dir = proc / std::to_string(pid);
  write_file(pid_dir / "cmdline", "/opt/axon/bin/" + executable + std::string("\0--flag\0", 8));
  write_file(pid_dir / "stat", stat_line(pid, executable, utime, stime, start_time));
  write_file(pid_dir / "statm", "1000 " + std::to_string(rss_pages) + " 0 0 0 0 0\n");
  write_file(
    pid_dir / "io",
    "rchar: 0\n"
    "wchar: 0\n"
    "read_bytes: " +
      std::to_string(read_bytes) +
      "\n"
      "write_bytes: " +
      std::to_string(write_bytes) + "\n"
  );
}

}  // namespace

int main() {
  const auto root = make_temp_dir("axon_system_processes");
  try {
    const auto proc = root / "proc";
    const auto agent = root / "agent";
    write_process(proc, 100, "axon-recorder", 10, 5, 1000, 25, 4096, 8192);
    write_process(proc, 200, "axon-transfer", 7, 3, 900, 30, 1024, 2048);
    write_process(proc, 300, "other-process", 1, 1, 1100, 5, 0, 0);
    write_process(proc, 400, "axon-rpc-test", 3, 2, 1200, 15, 512, 1024);
    write_file(agent / "demo_transfer.pid", "200\n");
    write_file(agent / "wrong_recorder.pid", "300\n");
    write_file(agent / "stale_transfer.pid", "999\n");
    write_file(agent / "stale_missing.pid", "998\n");

    auto rpc_failure_server = start_http_response_server(
      R"({"success":false,"message":"recorder error","data":{"state":"ERROR"}})"
    );

    axon::system::ProcessMonitorOptions options;
    options.proc_root = proc;
    options.targets = {
      {"recorder", "axon-recorder", {}, {}, {}, std::nullopt},
      {"transfer", "axon-transfer", {}, {agent / "*_transfer.pid"}, {}, std::nullopt},
      {"missing", "axon-missing", {}, {}, {}, std::nullopt},
      {"wrong_pid", "axon-recorder", agent / "wrong_recorder.pid", {}, {}, std::nullopt},
      {"stale_transfer", "axon-transfer", agent / "stale_transfer.pid", {}, {}, std::nullopt},
      {"stale_missing", "axon-missing", agent / "stale_missing.pid", {}, {}, std::nullopt},
      {"rpc_failed",
       "axon-rpc-test",
       {},
       {},
       {},
       axon::system::ProcessHttpProbeConfig{
         "http", "http://127.0.0.1:" + std::to_string(rpc_failure_server.port) + "/rpc/state", 350
       }},
    };

    axon::system::ProcessMonitor monitor(options);
    auto first = monitor.collect();
    require(first["recorder"]["status"].get<std::string>() == "healthy", "recorder status");
    require(first["recorder"]["pid"].get<int>() == 100, "recorder pid");
    require(first["recorder"]["source"].get<std::string>() == "process_match", "recorder source");
    require(
      first["recorder"]["resources"]["rss_bytes"].get<std::uint64_t>() ==
        25 * static_cast<std::uint64_t>(sysconf(_SC_PAGESIZE)),
      "recorder rss"
    );
    require(
      first["recorder"]["resources"]["io"]["read_bytes"].get<std::uint64_t>() == 4096,
      "recorder read bytes"
    );
    require(first["transfer"]["status"].get<std::string>() == "healthy", "transfer status");
    require(first["transfer"]["pid"].get<int>() == 200, "transfer pid");
    require(
      first["transfer"]["source"].get<std::string>() == "pid_file_candidate", "transfer source"
    );
    require(first["missing"]["status"].get<std::string>() == "unavailable", "missing status");
    require(first["wrong_pid"]["status"].get<std::string>() == "unhealthy", "wrong pid status");
    require(
      first["stale_transfer"]["status"].get<std::string>() == "healthy",
      "stale transfer should fall back to process match"
    );
    require(first["stale_transfer"]["pid"].get<int>() == 200, "stale transfer pid");
    require(
      first["stale_transfer"]["source"].get<std::string>() == "process_match",
      "stale transfer source"
    );
    require(
      first["stale_missing"]["status"].get<std::string>() == "unavailable", "stale missing status"
    );
    require(first["stale_missing"]["pid"].is_null(), "stale missing pid should be null");
    require(first["stale_missing"]["stale_pid"].get<int>() == 998, "stale missing stale pid");
    require(first["rpc_failed"]["status"].get<std::string>() == "unhealthy", "rpc failure status");
    require(
      first["rpc_failed"]["health"]["http_reachable"].get<bool>(),
      "rpc failure should have reachable HTTP"
    );
    require(
      first["rpc_failed"]["health"]["rpc_success"].get<bool>() == false,
      "rpc failure should expose RPC success false"
    );
    require(
      first["rpc_failed"]["health"]["reachable"].get<bool>() == false,
      "rpc failure should fail overall health"
    );
    require(
      first["rpc_failed"]["health"]["state"].get<std::string>() == "ERROR", "rpc failure state"
    );
    require(
      first["rpc_failed"]["message"].get<std::string>() == "recorder error", "rpc failure message"
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    write_process(proc, 100, "axon-recorder", 20, 10, 1000, 25, 4096, 8192);
    auto second = monitor.collect();
    require(
      second["recorder"]["resources"]["cpu_percent"].get<double>() > 0.0, "recorder cpu delta"
    );

    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::filesystem::remove_all(root);
    return 1;
  }
}

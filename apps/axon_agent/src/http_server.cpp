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
namespace agent {

namespace beast = boost::beast;
namespace http = boost::beast::http;
using tcp = boost::asio::ip::tcp;

HttpServer::HttpServer(std::string host, std::uint16_t port, AgentService& service)
  : host_(std::move(host)), port_(port), service_(service) {}

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
    std::cerr << "axon-agent HTTP server failed to start: " << ex.what() << std::endl;
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
        std::cerr << "axon-agent accept failed: " << ec.message() << std::endl;
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

  if (target == "/" || target == "/index.html") {
    return make_response(http::status::ok, "text/html; charset=utf-8", index_html(), request.version(), request.keep_alive());
  }

  if (target == "/health") {
    const nlohmann::json body = {{"success", true}, {"message", "ok"}};
    return make_response(http::status::ok, "application/json", body.dump(2), request.version(), request.keep_alive());
  }

  if (target == "/rpc/state" && method == "GET") {
    const auto rpc_response = service_.get_state();
    return make_response(
      rpc_response.success ? http::status::ok : http::status::bad_request, "application/json",
      rpc_response.to_json().dump(2), request.version(), request.keep_alive()
    );
  }

  if (target.rfind("/agent/rpc/", 0) == 0) {
    nlohmann::json params = nlohmann::json::object();
    if (!request.body().empty()) {
      try {
        params = nlohmann::json::parse(request.body());
      } catch (const std::exception& ex) {
        RpcResponse response{false, std::string("invalid json: ") + ex.what(), nullptr};
        return make_response(
          http::status::bad_request, "application/json", response.to_json().dump(2), request.version(),
          request.keep_alive()
        );
      }
    }

    auto rpc_response = route_rpc(target.substr(std::string("/agent/rpc/").size()), method, params);
    return make_response(
      rpc_response.success ? http::status::ok : http::status::bad_request, "application/json",
      rpc_response.to_json().dump(2), request.version(), request.keep_alive()
    );
  }

  return make_response(
    http::status::not_found, "application/json",
    nlohmann::json({{"success", false}, {"message", "not found"}}).dump(2), request.version(), request.keep_alive()
  );
}

RpcResponse HttpServer::route_rpc(const std::string& path, const std::string& method, const nlohmann::json& params) {
  if (path == "state" && method == "GET") {
    return service_.get_state();
  }
  if (path == "report" && method == "GET") {
    return service_.get_report();
  }
  if (path == "profiles" && method == "GET") {
    return service_.list_profiles();
  }
  if (path == "profile/select" && method == "POST") {
    return service_.select_profile(params);
  }
  if (path == "process/start" && method == "POST") {
    return service_.start_process(params);
  }
  if (path == "process/stop" && method == "POST") {
    return service_.stop_process(params, false);
  }
  if (path == "process/force_stop" && method == "POST") {
    return service_.stop_process(params, true);
  }
  if (path == "process/log" && method == "POST") {
    return service_.read_process_log(params);
  }
  return {false, "unknown agent rpc: " + path, nullptr};
}

HttpServer::Response HttpServer::make_response(
  http::status status, const std::string& content_type, const std::string& body, unsigned int version, bool keep_alive
) {
  Response response{status, version};
  response.set(http::field::server, "axon-agent");
  response.set(http::field::content_type, content_type);
  response.keep_alive(keep_alive);
  response.body() = body;
  response.prepare_payload();
  return response;
}

std::string HttpServer::index_html() {
  return R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Axon Agent</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; background: #0f172a; color: #e2e8f0; }
    button, input, select { margin: 0.25rem; padding: 0.45rem 0.7rem; }
    input, select { background: #0f172a; border: 1px solid #334155; color: #e2e8f0; }
    pre { background: #111827; padding: 1rem; border-radius: 8px; overflow: auto; }
    .card { background: #1e293b; padding: 1rem; border-radius: 10px; margin-bottom: 1rem; }
    .log { min-height: 12rem; white-space: pre-wrap; }
  </style>
</head>
<body>
  <h1>Axon Agent</h1>
  <div class="card">
    <button onclick="refresh()">Refresh</button>
    <select id="profiles"></select>
    <button onclick="selectProfile()">Select Profile</button>
  </div>
  <div class="card">
    <button onclick="startProcess('robot_startup')">Start Robot</button>
    <button onclick="startProcess('recorder')">Start Recorder</button>
    <button onclick="startProcess('transfer')">Start Transfer</button>
    <button onclick="stopProcess('robot_startup', false)">Stop Robot</button>
    <button onclick="stopProcess('recorder', false)">Stop Recorder</button>
    <button onclick="stopProcess('transfer', false)">Stop Transfer</button>
  </div>
  <div class="card">
    <select id="logProcess">
      <option value="robot_startup">robot_startup</option>
      <option value="recorder">recorder</option>
      <option value="transfer">transfer</option>
    </select>
    <select id="logStream">
      <option value="stdout">stdout</option>
      <option value="stderr">stderr</option>
    </select>
    <input id="logTail" type="number" min="0" max="4194304" value="65536">
    <button onclick="readLog()">Read Log</button>
  </div>
  <pre id="logOutput" class="log"></pre>
  <pre id="output"></pre>
  <script>
    async function rpc(path, method = 'GET', body = undefined) {
      const res = await fetch('/agent/rpc/' + path, {
        method,
        headers: {'Content-Type': 'application/json'},
        body: body ? JSON.stringify(body) : undefined
      });
      const json = await res.json();
      document.getElementById('output').textContent = JSON.stringify(json, null, 2);
      return json;
    }
    async function refresh() {
      const profiles = await rpc('profiles');
      const select = document.getElementById('profiles');
      select.innerHTML = '';
      for (const profile of profiles.data.profiles) {
        const option = document.createElement('option');
        option.value = profile.profile_id;
        option.textContent = `${profile.profile_id} (${profile.robot_model})`;
        select.appendChild(option);
      }
      await rpc('state');
    }
    async function selectProfile() {
      await rpc('profile/select', 'POST', {profile_id: document.getElementById('profiles').value});
    }
    async function startProcess(process_id) {
      await rpc('process/start', 'POST', {process_id});
    }
    async function stopProcess(process_id, force) {
      await rpc(force ? 'process/force_stop' : 'process/stop', 'POST', {process_id});
    }
    async function readLog() {
      const process_id = document.getElementById('logProcess').value;
      const stream = document.getElementById('logStream').value;
      const tail_bytes = Number(document.getElementById('logTail').value || 0);
      const result = await rpc('process/log', 'POST', {process_id, stream, tail_bytes});
      const data = result.data || {};
      document.getElementById('logOutput').textContent = result.success ? (data.content || '') : result.message;
    }
    refresh();
  </script>
</body>
</html>)HTML";
}

}  // namespace agent
}  // namespace axon

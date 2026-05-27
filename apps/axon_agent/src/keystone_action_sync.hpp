// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_AGENT_KEYSTONE_ACTION_SYNC_HPP
#define AXON_AGENT_KEYSTONE_ACTION_SYNC_HPP

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "agent_service.hpp"

namespace axon {
namespace agent {

struct KeystoneActionSyncConfig {
  bool enabled = false;
  std::string base_url;
  std::string auth_token;
  std::string robot_id;
  std::string websocket_url;
  int pending_limit = 10;
  std::chrono::milliseconds http_timeout{3000};
  std::chrono::milliseconds websocket_reconnect_interval{5000};
  std::chrono::milliseconds catalog_sync_interval{60000};
  std::chrono::milliseconds poll_interval{5000};
  std::chrono::milliseconds min_backoff{1000};
  std::chrono::milliseconds max_backoff{30000};
};

struct KeystoneHttpResponse {
  bool ok = false;
  int status_code = 0;
  nlohmann::json body = nullptr;
  std::string error;
};

class KeystoneActionTransport {
public:
  virtual ~KeystoneActionTransport() = default;

  virtual KeystoneHttpResponse get(const std::string& path, std::chrono::milliseconds timeout) = 0;
  virtual KeystoneHttpResponse put_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
  ) = 0;
  virtual KeystoneHttpResponse post_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
  ) = 0;
};

class KeystoneActionHttpTransport : public KeystoneActionTransport {
public:
  KeystoneActionHttpTransport(std::string base_url, std::string auth_token);

  KeystoneHttpResponse get(const std::string& path, std::chrono::milliseconds timeout) override;
  KeystoneHttpResponse put_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
  ) override;
  KeystoneHttpResponse post_json(
    const std::string& path, const nlohmann::json& body, std::chrono::milliseconds timeout
  ) override;

private:
  KeystoneHttpResponse request(
    const std::string& method, const std::string& path, const nlohmann::json* body,
    std::chrono::milliseconds timeout
  );

  std::string scheme_;
  std::string host_;
  std::string port_;
  std::string base_path_;
  std::string auth_token_;
  std::string parse_error_;
};

class KeystoneActionNotificationClient;

class KeystoneActionSync {
public:
  KeystoneActionSync(
    AgentService& service, KeystoneActionSyncConfig config,
    std::unique_ptr<KeystoneActionTransport> transport = nullptr
  );
  ~KeystoneActionSync();

  KeystoneActionSync(const KeystoneActionSync&) = delete;
  KeystoneActionSync& operator=(const KeystoneActionSync&) = delete;

  bool start();
  void stop();
  bool is_running() const;

  bool sync_catalog_once(std::string* error);
  bool poll_once(std::string* error);
  bool handle_notification_once(const nlohmann::json& notification, std::string* error);
  nlohmann::json status_to_json() const;

private:
  void run();
  std::string robot_id_or_error(std::string* error);
  bool report_status(
    const std::string& robot_id, const std::string& request_id, const std::string& status,
    const nlohmann::json& execution, std::string* error
  );
  bool process_request(
    const std::string& robot_id, const nlohmann::json& pending, std::string* error
  );
  bool process_request_id(
    const std::string& robot_id, const std::string& request_id, std::string* error
  );
  bool execute_detail(
    const std::string& robot_id, const std::string& request_id, const nlohmann::json& detail,
    std::string* error
  );
  void record_success(const std::string& timestamp_field);
  void record_failure(const std::string& error);
  std::chrono::milliseconds current_backoff() const;

  AgentService& service_;
  KeystoneActionSyncConfig config_;
  std::unique_ptr<KeystoneActionTransport> transport_;
  std::unique_ptr<KeystoneActionNotificationClient> notification_client_;
  std::unique_ptr<std::thread> thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex stats_mutex_;
  std::uint64_t catalog_sync_count_ = 0;
  std::uint64_t poll_count_ = 0;
  std::uint64_t notification_count_ = 0;
  std::uint64_t action_requests_executed_ = 0;
  std::uint64_t status_updates_sent_ = 0;
  std::uint64_t consecutive_failures_ = 0;
  std::chrono::milliseconds backoff_delay_{0};
  std::string last_error_;
  std::string last_catalog_sync_at_;
  std::string last_poll_at_;
  std::string last_notification_at_;
};

std::string keystone_action_sync_now_iso8601();
std::string keystone_action_status_from_local(const std::string& local_status);
std::string keystone_url_encode_component(const std::string& value);

}  // namespace agent
}  // namespace axon

#endif  // AXON_AGENT_KEYSTONE_ACTION_SYNC_HPP

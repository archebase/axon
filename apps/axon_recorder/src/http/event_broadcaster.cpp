// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "event_broadcaster.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace axon {
namespace recorder {

EventBroadcaster::EventBroadcaster(WebSocketServer& ws_server)
    : ws_server_(ws_server)
    , stats_running_(false) {}

EventBroadcaster::~EventBroadcaster() {
  stop_stats_broadcast();
}

void EventBroadcaster::start_stats_broadcast(std::chrono::milliseconds interval) {
  if (stats_running_.load()) {
    return;
  }

  stats_interval_ = interval;
  stats_running_.store(true);

  // Start the timer thread
  stats_thread_ = std::thread(&EventBroadcaster::stats_timer_thread, this);

  std::cout << "[EventBroadcaster] Starting stats broadcast every " << interval.count() << "ms"
            << std::endl;
}

void EventBroadcaster::stop_stats_broadcast() {
  if (!stats_running_.load()) {
    return;
  }

  stats_running_.store(false);

  // Notify the thread to wake up and exit
  stats_cv_.notify_all();

  // Wait for thread to finish
  if (stats_thread_.joinable()) {
    stats_thread_.join();
  }

  std::cout << "[EventBroadcaster] Stopped stats broadcast" << std::endl;
}

void EventBroadcaster::broadcast_state_change(
  RecorderState from, RecorderState to, const std::string& task_id
) {
  nlohmann::json data;
  data["previous"] = state_to_string(from);
  data["current"] = state_to_string(to);
  data["task_id"] = task_id;

  ws_server_.broadcast("state", data);
}

void EventBroadcaster::broadcast_config_change(const TaskConfig* config) {
  nlohmann::json data;

  if (config != nullptr) {
    data["action"] = "set";
    data["task_config"] = {
      {"task_id", config->task_id},
      {"device_id", config->device_id},
      {"scene", config->scene},
      {"topics", config->topics}
    };
  } else {
    data["action"] = "clear";
  }

  ws_server_.broadcast("config", data);
}

void EventBroadcaster::broadcast_log(
  const std::string& level, const std::string& message, const nlohmann::json& details
) {
  nlohmann::json data;
  data["level"] = level;
  data["message"] = message;
  if (!details.is_null()) {
    data["details"] = details;
  }

  ws_server_.broadcast("log", data);
}

void EventBroadcaster::broadcast_error(
  const std::string& code, const std::string& message, const std::string& severity,
  const nlohmann::json& details
) {
  nlohmann::json data;
  data["code"] = code;
  data["message"] = message;
  data["severity"] = severity;
  if (!details.is_null()) {
    data["details"] = details;
  }

  ws_server_.broadcast("error", data);
}

void EventBroadcaster::broadcast_stats(const nlohmann::json& stats) {
  ws_server_.broadcast("stats", stats);
}

void EventBroadcaster::set_stats_callback(StatsCallback callback) {
  stats_callback_ = std::move(callback);
}

void EventBroadcaster::set_state_callback(StateCallback callback) {
  state_callback_ = std::move(callback);
}

void EventBroadcaster::set_config_callback(ConfigCallback callback) {
  config_callback_ = std::move(callback);
}

void EventBroadcaster::stats_timer_thread() {
  while (stats_running_.load()) {
    // Broadcast stats if callback is set
    if (stats_callback_) {
      auto stats = stats_callback_();
      if (!stats.is_null()) {
        broadcast_stats(stats);
      }
    }

    // Wait for the interval or until stopped
    std::unique_lock<std::mutex> lock(stats_mutex_);
    stats_cv_.wait_for(lock, stats_interval_, [this] {
      return !stats_running_.load();
    });
  }
}

std::string EventBroadcaster::get_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::ostringstream oss;
  oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%dT%H:%M:%S");
  oss << "." << std::setfill('0') << std::setw(3) << ms.count() << "Z";

  return oss.str();
}

}  // namespace recorder
}  // namespace axon

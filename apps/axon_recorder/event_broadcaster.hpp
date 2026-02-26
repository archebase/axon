// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_EVENT_BROADCASTER_HPP
#define AXON_RECORDER_EVENT_BROADCASTER_HPP

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "state_machine.hpp"
#include "task_config.hpp"
#include "websocket_server.hpp"

namespace axon {
namespace recorder {

/**
 * Forward declaration
 */
class WebSocketServer;

/**
 * EventBroadcaster - broadcasts events to all connected WebSocket clients
 *
 * Provides methods to broadcast state changes, statistics, configuration updates,
 * log events, and errors. Supports periodic stats broadcasting.
 */
class EventBroadcaster {
public:
  /**
   * Constructor
   * @param ws_server Reference to WebSocket server
   */
  explicit EventBroadcaster(WebSocketServer& ws_server);

  // Non-copyable
  EventBroadcaster(const EventBroadcaster&) = delete;
  EventBroadcaster& operator=(const EventBroadcaster&) = delete;

  /**
   * Destructor - stops all timers
   */
  ~EventBroadcaster();

  /**
   * Start periodic statistics broadcast
   * @param interval Broadcast interval
   */
  void start_stats_broadcast(std::chrono::milliseconds interval);

  /**
   * Stop periodic statistics broadcast
   */
  void stop_stats_broadcast();

  /**
   * Broadcast state change
   * @param from Previous state
   * @param to New state
   * @param task_id Current task ID
   */
  void broadcast_state_change(RecorderState from, RecorderState to, const std::string& task_id);

  /**
   * Broadcast configuration change
   * @param config Task configuration (nullptr to indicate clear)
   */
  void broadcast_config_change(const TaskConfig* config);

  /**
   * Broadcast log event
   * @param level Log level (debug/info/warning/error)
   * @param message Log message
   * @param details Additional details
   */
  void broadcast_log(
    const std::string& level, const std::string& message, const nlohmann::json& details = {}
  );

  /**
   * Broadcast error
   * @param code Error code
   * @param message Error message
   * @param severity Error severity (info/warning/critical)
   * @param details Additional error details
   */
  void broadcast_error(
    const std::string& code, const std::string& message, const std::string& severity,
    const nlohmann::json& details = {}
  );

  /**
   * Broadcast statistics
   * @param stats Statistics JSON
   */
  void broadcast_stats(const nlohmann::json& stats);

  /**
   * Register callback for statistics
   * @param callback Function that returns current stats
   */
  using StatsCallback = std::function<nlohmann::json()>;
  void set_stats_callback(StatsCallback callback);

  /**
   * Register callback for current state
   * @param callback Function that returns current state
   */
  using StateCallback = std::function<std::pair<RecorderState, std::string>()>;
  void set_state_callback(StateCallback callback);

  /**
   * Register callback for current task config
   * @param callback Function that returns current config pointer
   */
  using ConfigCallback = std::function<const TaskConfig*()>;
  void set_config_callback(ConfigCallback callback);

private:
  /**
   * Stats broadcast timer thread function
   */
  void stats_timer_thread();

  /**
   * Get current timestamp in ISO 8601 format
   */
  std::string get_timestamp();

  WebSocketServer& ws_server_;

  std::atomic<bool> stats_running_{false};
  std::chrono::milliseconds stats_interval_;
  std::thread stats_thread_;
  std::mutex stats_mutex_;
  std::condition_variable stats_cv_;

  StatsCallback stats_callback_;
  StateCallback state_callback_;
  ConfigCallback config_callback_;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_EVENT_BROADCASTER_HPP

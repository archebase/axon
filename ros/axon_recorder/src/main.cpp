/**
 * @file main.cpp
 * @brief Main entry point for the axon_recorder node
 *
 * This file contains only the main() function. The RecorderNode implementation
 * is in recorder_node.cpp which is part of axon_recorder_lib.
 */

#include "recorder_node.hpp"
#include "version.hpp"

#include <csignal>

// Logging infrastructure
#define AXON_LOG_COMPONENT "main"
#include <axon_log_macros.hpp>
#include <axon_log_init.hpp>

#if defined(AXON_ROS1)
#include <ros/ros.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif

namespace {
// Global flag for signal handling
volatile std::sig_atomic_t g_shutdown_requested = 0;
volatile std::sig_atomic_t g_received_signal = 0;

void signal_handler(int signum) {
  // IMPORTANT: Signal handlers must only use async-signal-safe functions.
  // Do NOT call AXON_LOG_* here - Boost.Log uses mutexes and memory allocation
  // which can deadlock if a signal arrives while logging is in progress.
  // Simply set flags and return. Logging happens after main loop exits.
  g_received_signal = signum;
  g_shutdown_requested = 1;
  
  // Note: ros::shutdown() and rclcpp::shutdown() are not strictly async-signal-safe,
  // but they're designed to be callable from signal handlers in practice.
#if defined(AXON_ROS1)
  ros::shutdown();
#elif defined(AXON_ROS2)
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
#endif
}
}  // namespace

int main(int argc, char** argv) {
  // Initialize logging first (console only initially, file sink added via config)
  axon::logging::LoggingConfig log_config;
  log_config.console_enabled = true;
  log_config.console_colors = true;
  log_config.console_level = axon::logging::severity_level::info;
  log_config.file_enabled = false;  // Can be enabled via config file later
  
  axon::logging::init_logging(log_config);
  
  AXON_LOG_INFO("axon_recorder starting (MCAP backend)" << axon::logging::kv("version", AXON_RECORDER_VERSION));

  // Use a block scope to ensure RecorderNode is fully destroyed before main() returns
  int exit_code = 0;
  {
    // Use factory method to create RecorderNode as shared_ptr
    // This enables shared_from_this() for dependency injection
    auto node = axon::recorder::RecorderNode::create();

    if (!node->initialize(argc, argv)) {
      AXON_LOG_FATAL("Initialization failed");
      axon::logging::shutdown_logging();
      return 1;
    }

    // Install custom signal handlers AFTER rclcpp::init() to override ROS defaults
    AXON_LOG_DEBUG("Installing signal handlers...");
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    AXON_LOG_INFO("Initialization complete, starting main loop");

    // run() blocks until rclcpp::shutdown() is called (by signal or otherwise)
    node->run();

    // Log the signal that triggered shutdown (safe to log here, outside signal handler)
    if (g_received_signal != 0) {
      AXON_LOG_INFO("Received signal, shutting down" << ::axon::logging::kv("signal", static_cast<int>(g_received_signal)));
    }
    AXON_LOG_INFO("Main loop exited, calling shutdown()");

    // Ensure shutdown is called - this writes the stats file
    // This runs AFTER spin() returns due to SIGTERM/SIGINT
    node->shutdown();

    AXON_LOG_INFO("Node shutdown complete");
  }  // RecorderNode shared_ptr released here, destructor runs before main() exits

  AXON_LOG_INFO("Exiting normally");
  
  // Shutdown logging (flushes all pending log messages)
  axon::logging::shutdown_logging();
  
  return exit_code;
}

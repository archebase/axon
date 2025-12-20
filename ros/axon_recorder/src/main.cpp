/**
 * @file main.cpp
 * @brief Main entry point for the axon_recorder node
 *
 * This file contains only the main() function. The RecorderNode implementation
 * is in recorder_node.cpp which is part of axon_recorder_lib.
 */

#include "recorder_node.hpp"

#include <csignal>
#include <iostream>

#if defined(AXON_ROS1)
#include <ros/ros.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif

namespace {
// Global flag for signal handling
volatile std::sig_atomic_t g_shutdown_requested = 0;

void signal_handler(int signum) {
  std::cerr << "\n[axon_recorder] Received signal " << signum << ", requesting shutdown..." << std::endl;
  g_shutdown_requested = 1;
  
  // Also trigger ROS shutdown
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
  std::cerr << "[axon_recorder] main() starting (MCAP backend)" << std::endl;

  // Use a block scope to ensure RecorderNode is fully destroyed before main() returns
  {
    axon::recorder::RecorderNode node;

    if (!node.initialize(argc, argv)) {
      std::cerr << "[axon_recorder] Initialization failed" << std::endl;
      return 1;
    }

    // Install custom signal handlers AFTER rclcpp::init() to override ROS defaults
    std::cerr << "[axon_recorder] Installing signal handlers..." << std::endl;
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGINT, signal_handler);

    std::cerr << "[axon_recorder] Initialization complete, calling run()" << std::endl;

    // run() blocks until rclcpp::shutdown() is called (by signal or otherwise)
    node.run();

    std::cerr << "[axon_recorder] run() returned, calling shutdown()" << std::endl;

    // Ensure shutdown is called - this writes the stats file
    // This runs AFTER spin() returns due to SIGTERM/SIGINT
    node.shutdown();

    std::cerr << "[axon_recorder] Node destroyed" << std::endl;
  }  // RecorderNode destructor runs here, before main() exits

  std::cerr << "[axon_recorder] main() exiting normally" << std::endl;
  return 0;
}


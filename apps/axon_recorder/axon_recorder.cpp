// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "config_parser.hpp"
#include "recorder.hpp"
#include "version.hpp"

namespace axon {
namespace recorder {

namespace {
// Global recorder instance for signal handling
AxonRecorder* g_recorder = nullptr;
std::atomic<bool> g_should_exit(false);

void signal_handler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    std::cout << "\nReceived signal " << signal << ", stopping recorder..." << std::endl;
    g_should_exit.store(true);

    if (g_recorder) {
      // Stop HTTP server if running
      if (g_recorder->is_http_server_running()) {
        g_recorder->stop_http_server();
      }
      // Stop recording if running
      if (g_recorder->is_running()) {
        g_recorder->stop();
      }
    }
  }
}

void print_usage(const char* program_name) {
  std::cout
    << "Usage: " << program_name << " [OPTIONS]\n"
    << "\n"
    << "Axon Recorder - Plugin-based ROS message recorder\n"
    << "Version: " << axon::recorder::get_version() << "\n"
    << "\n"
    << "Options:\n"
    << "  --simple               Simple mode: start recording immediately without HTTP RPC\n"
    << "  --output FILE          Output file path (default: auto-generated timestamp)\n"
    << "  --config PATH          Path to YAML configuration file\n"
    << "  --plugin PATH         Path to ROS plugin shared library (.so)\n"
    << "  --path PATH           Output directory path (default: .)\n"
    << "  --profile PROFILE     ROS profile: ros1 or ros2 (default: ros2)\n"
    << "  --compression ALG     Compression: none, zstd, lz4 (default: zstd)\n"
    << "  --level LEVEL         Compression level (default: 3)\n"
    << "  --queue-size SIZE     Message queue capacity (default: 1024)\n"
    << "  --version             Show version information\n"
    << "  --help                Show this help message\n"
    << "\n"
    << "Simple Mode:\n"
    << "  When --simple is specified, the recorder starts immediately without waiting for\n"
    << "  HTTP RPC commands. The output filename is either:\n"
    << "    - Specified via --output FILE\n"
    << "    - Auto-generated as YYYYMMDD_HHMMSS.mcap (e.g., 20240102_143000.mcap)\n"
    << "\n"
    << "  In simple mode, topics must be configured either via config file subscriptions\n"
    << "  or CLI parameters. No HTTP RPC server is started.\n"
    << "\n"
    << "HTTP RPC Server:\n"
    << "  Without --simple, the recorder starts an HTTP RPC server on port 8080 for remote "
       "control.\n"
    << "  The recorder starts in IDLE state and waits for RPC commands.\n"
    << "  Available endpoints:\n"
    << "    POST /rpc/config      - Set task configuration (IDLE->READY)\n"
    << "    POST /rpc/begin       - Start recording (READY->RECORDING)\n"
    << "    POST /rpc/finish      - Finish recording, return to IDLE (RECORDING/PAUSED->IDLE)\n"
    << "    POST /rpc/quit        - Stop recording and exit program (saves data first)\n"
    << "    POST /rpc/pause       - Pause recording (RECORDING->PAUSED)\n"
    << "    POST /rpc/resume      - Resume recording (PAUSED->RECORDING)\n"
    << "    POST /rpc/cancel      - Cancel recording (RECORDING/PAUSED->IDLE)\n"
    << "    POST /rpc/clear       - Clear config (READY->IDLE)\n"
    << "    GET  /rpc/state       - Get current state\n"
    << "    GET  /rpc/stats       - Get recording statistics\n"
    << "    GET  / or /health     - Health check\n"
    << "\n"
    << "Configuration File:\n"
    << "  If --config is provided, most options can be specified in a YAML file.\n"
    << "  Command-line arguments OVERRIDE config file values.\n"
    << "  Example config file structure:\n"
    << "  plugin:\n"
    << "    path: /path/to/plugin.so\n"
    << "  dataset:\n"
    << "    path: /data/recordings\n"
    << "  subscriptions:\n"
    << "    - name: /camera/image\n"
    << "      message_type: sensor_msgs/msg/Image\n"
    << "      batch_size: 300\n"
    << "      flush_interval_ms: 10000\n"
    << "  recording:\n"
    << "    profile: ros2\n"
    << "    compression: zstd\n"
    << "    compression_level: 3\n"
    << "\n"
    << "Output File Naming:\n"
    << "  Simple mode (--simple):\n"
    << "    - --output my_recording.mcap  -> Use specified filename\n"
    << "    - No --output                  -> Auto-generated: YYYYMMDD_HHMMSS.mcap\n"
    << "  HTTP RPC mode:\n"
    << "    Output files are named as: <dataset.path>/<task_id>.mcap\n"
    << "    The task_id is provided via the /rpc/config endpoint.\n"
    << "\n"
    << "Examples:\n"
    << "  # Simple mode with auto-generated filename\n"
    << "  " << program_name
    << " --simple --plugin ./ros2_plugin.so --config config/default_config_ros2.yaml\n"
    << "\n"
    << "  # Simple mode with custom filename\n"
    << "  " << program_name
    << " --simple --output /data/my_recording.mcap --plugin ./ros2_plugin.so\n"
    << "\n"
    << "  # Use config file only\n"
    << "  " << program_name << " --config config/default_config_ros2.yaml\n"
    << "\n"
    << "  # Override plugin path from config file\n"
    << "  " << program_name << " --config config/default_config_ros2.yaml \\\n"
    << "    --plugin /custom/path/to/libaxon_ros2_plugin.so\n"
    << std::endl;
}

void print_statistics(const AxonRecorder::Statistics& stats) {
  std::cout << "\n=== Recording Statistics ===\n"
            << "Messages received: " << stats.messages_received << "\n"
            << "Messages written:  " << stats.messages_written << "\n"
            << "Messages dropped:  " << stats.messages_dropped << "\n"
            << "Bytes written:     " << stats.bytes_written << "\n"
            << std::endl;
}

/**
 * Generate timestamp-based filename
 * Format: YYYYMMDD_HHMMSS.mcap (e.g., 20240102_143000.mcap)
 */
std::string generate_timestamp_filename() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);

  std::tm tm_buf;
#if defined(_WIN32) || defined(_WIN64)
  localtime_s(&tm_buf, &time_t_now);
#else
  localtime_r(&time_t_now, &tm_buf);
#endif

  std::stringstream ss;
  ss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return ss.str() + ".mcap";
}

}  // namespace

}  // namespace recorder
}  // namespace axon

int main(int argc, char* argv[]) {
  using namespace axon::recorder;

  // Check for help and version flags
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_usage(argv[0]);
      return 0;
    } else if (strcmp(argv[i], "--version") == 0) {
      std::cout << get_version_info() << std::endl;
      return 0;
    }
  }

  // Step 1: Parse command line arguments to extract config file path and CLI overrides
  std::string config_file;
  std::string cli_plugin_path;
  std::string cli_dataset_path;
  std::string cli_profile;
  std::string cli_compression;
  std::string cli_output_file;
  int cli_compression_level = -1;
  size_t cli_queue_capacity = 0;
  bool simple_mode = false;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--simple") == 0) {
      simple_mode = true;
    } else if (strcmp(argv[i], "--output") == 0) {
      if (i + 1 < argc) {
        cli_output_file = argv[++i];
      } else {
        std::cerr << "Error: --output requires a file argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--config") == 0) {
      if (i + 1 < argc) {
        config_file = argv[++i];
      } else {
        std::cerr << "Error: --config requires a file argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--plugin") == 0) {
      if (i + 1 < argc) {
        cli_plugin_path = argv[++i];
      } else {
        std::cerr << "Error: --plugin requires a path argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--path") == 0) {
      if (i + 1 < argc) {
        cli_dataset_path = argv[++i];
      } else {
        std::cerr << "Error: --path requires a directory argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--profile") == 0) {
      if (i + 1 < argc) {
        cli_profile = argv[++i];
      } else {
        std::cerr << "Error: --profile requires a profile argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--compression") == 0) {
      if (i + 1 < argc) {
        cli_compression = argv[++i];
      } else {
        std::cerr << "Error: --compression requires an algorithm argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--level") == 0) {
      if (i + 1 < argc) {
        cli_compression_level = std::atoi(argv[++i]);
      } else {
        std::cerr << "Error: --level requires a number argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--queue-size") == 0) {
      if (i + 1 < argc) {
        cli_queue_capacity = static_cast<size_t>(std::atol(argv[++i]));
      } else {
        std::cerr << "Error: --queue-size requires a number argument" << std::endl;
        return 1;
      }
    } else {
      std::cerr << "Error: Unknown argument: " << argv[i] << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }

  // Step 2: Load configuration file if specified
  RecorderConfig config;
  if (!config_file.empty()) {
    ConfigParser parser;
    if (!parser.load_from_file(config_file, config)) {
      std::cerr << "Error: Failed to load config file '" << config_file
                << "': " << parser.get_last_error() << std::endl;
      return 1;
    }
  }

  // Step 3: Apply CLI argument overrides (CLI takes precedence over config file)
  if (!cli_plugin_path.empty()) {
    config.plugin_path = cli_plugin_path;
  }
  if (!cli_dataset_path.empty()) {
    config.dataset.path = cli_dataset_path;
  }
  if (!cli_profile.empty()) {
    config.profile = cli_profile;
  }
  if (!cli_compression.empty()) {
    config.compression = cli_compression;
  }
  if (cli_compression_level >= 0) {
    config.compression_level = cli_compression_level;
  }
  if (cli_queue_capacity > 0) {
    config.queue_capacity = cli_queue_capacity;
  }

  // Handle output filename in simple mode
  if (simple_mode) {
    if (!cli_output_file.empty()) {
      // User specified output file
      config.output_file = cli_output_file;
    } else {
      // Auto-generate timestamp-based filename
      config.output_file = generate_timestamp_filename();
    }
  }

  // Validate required arguments
  if (config.plugin_path.empty()) {
    std::cerr << "Error: --plugin (or plugin.path in config file) is required" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  // Print configuration
  std::cout << "Axon Recorder Configuration:\n"
            << "  Mode:        " << (simple_mode ? "Simple (direct recording)" : "HTTP RPC") << "\n"
            << "  Plugin:      " << config.plugin_path << "\n"
            << "  Output:      " << config.output_file << "\n"
            << "  Dataset:     " << config.dataset.path << "\n"
            << "  Profile:     " << config.profile << "\n"
            << "  Compression: " << config.compression << " level " << config.compression_level
            << "\n"
            << "  Queue size:  " << config.queue_capacity << "\n";

  // Show topics if configured
  if (!config.subscriptions.empty()) {
    std::cout << "  Topics:\n";
    for (const auto& sub : config.subscriptions) {
      std::cout << "    - " << sub.topic_name << " (" << sub.message_type << ")\n";
    }
  } else {
    std::cout << "  Topics:      (no subscriptions configured)" << std::endl;
  }
  std::cout << std::endl;

  // Setup signal handlers
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Create and initialize recorder
  AxonRecorder recorder;
  g_recorder = &recorder;

  if (!recorder.initialize(config)) {
    std::cerr << "Error: Failed to initialize recorder: " << recorder.get_last_error() << std::endl;
    return 1;
  }

  if (simple_mode) {
    // Simple mode: start recording immediately without HTTP RPC
    std::cout << "Starting recording in simple mode..." << std::endl;

    if (!recorder.start()) {
      std::cerr << "Error: Failed to start recording: " << recorder.get_last_error() << std::endl;
      return 1;
    }

    std::cout << "Recording started. Press Ctrl+C to stop." << std::endl;

    // Wait for quit signal
    while (!g_should_exit.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Print statistics if recording is active
      if (recorder.is_running()) {
        auto stats = recorder.get_statistics();
        if (stats.messages_written > 0) {
          std::cout << "\rMessages: " << stats.messages_written << " written, "
                    << stats.messages_received << " received, " << stats.messages_dropped
                    << " dropped   " << std::flush;
        }
      }
    }

    std::cout << "\nStopping recording..." << std::endl;

    // Stop recording
    recorder.stop();

    // Print final statistics
    auto final_stats = recorder.get_statistics();
    print_statistics(final_stats);

    return 0;
  }

  // HTTP RPC mode: start HTTP server and wait for commands
  // Start HTTP RPC server
  std::cout << "Starting HTTP RPC server on " << config.http_server.host << ":"
            << config.http_server.port << "..." << std::endl;

  // Register shutdown callback to set exit flag when /rpc/quit is called
  recorder.set_shutdown_callback([]() {
    std::cout << "\nReceived quit request via HTTP RPC..." << std::endl;
    g_should_exit.store(true);
  });

  if (!recorder.start_http_server(config.http_server.host, config.http_server.port)) {
    std::cerr << "Warning: Failed to start HTTP server: " << recorder.get_last_error() << std::endl;
    std::cerr << "Continuing without HTTP RPC control..." << std::endl;
  } else {
    std::cout << "HTTP RPC server listening on http://" << config.http_server.host << ":"
              << config.http_server.port << std::endl;
  }

  // Recorder starts in IDLE state, waiting for RPC commands
  std::cout << "Recorder ready (current state: " << recorder.get_state_string() << ")" << std::endl;
  std::cout << "Waiting for RPC commands. Use Ctrl+C to quit." << std::endl;

  // Wait for quit signal (recording controlled via RPC)
  while (!g_should_exit.load()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Print statistics if recording is active
    if (recorder.is_running()) {
      auto stats = recorder.get_statistics();
      if (stats.messages_written > 0) {
        std::cout << "\rMessages: " << stats.messages_written << " written, "
                  << stats.messages_received << " received, " << stats.messages_dropped
                  << " dropped   " << std::flush;
      }
    }
  }

  std::cout << "\nStopping recording..." << std::endl;

  // Stop recording
  recorder.stop();

  // Stop HTTP server
  if (recorder.is_http_server_running()) {
    std::cout << "Stopping HTTP RPC server..." << std::endl;
    recorder.stop_http_server();
  }

  // Print final statistics
  auto final_stats = recorder.get_statistics();
  print_statistics(final_stats);

  return 0;
}

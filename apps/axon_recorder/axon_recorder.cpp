#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

#include "config_parser.hpp"
#include "recorder.hpp"

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
    << "\n"
    << "Options:\n"
    << "  --config PATH          Path to YAML configuration file\n"
    << "  --plugin PATH         Path to ROS plugin shared library (.so)\n"
    << "  --output FILE         Output MCAP file path (default: output.mcap)\n"
    << "  --topic NAME          Subscribe to topic (can be used multiple times)\n"
    << "  --type TYPE           Message type for last --topic (e.g., "
       "sensor_msgs/msg/Image)\n"
    << "  --profile PROFILE     ROS profile: ros1 or ros2 (default: ros2)\n"
    << "  --compression ALG     Compression: none, zstd, lz4 (default: zstd)\n"
    << "  --level LEVEL         Compression level (default: 3)\n"
    << "  --queue-size SIZE     Message queue capacity (default: 1024)\n"
    << "  --help                Show this help message\n"
    << "\n"
    << "HTTP RPC Server:\n"
    << "  The recorder starts an HTTP RPC server on port 8080 for remote control.\n"
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
    << "    output_file: recording.mcap\n"
    << "    queue_size: 8192\n"
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
    << "Examples:\n"
    << "  # Use config file only\n"
    << "  " << program_name << " --config config/default_config_ros2.yaml\n"
    << "\n"
    << "  # Override plugin path from config file\n"
    << "  " << program_name << " --config config/default_config_ros2.yaml \\\n"
    << "    --plugin /custom/path/to/libaxon_ros2_plugin.so\n"
    << "\n"
    << "  # Override output file and compression level\n"
    << "  " << program_name << " --config config/default_config_ros2.yaml \\\n"
    << "    --output /tmp/test.mcap --level 5\n"
    << "\n"
    << "  # CLI only (no config file)\n"
    << "  " << program_name << " --plugin ./ros2_plugin.so \\\n"
    << "    --topic /camera/image_raw --type sensor_msgs/msg/Image \\\n"
    << "    --output recording.mcap\n"
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

}  // namespace

}  // namespace recorder
}  // namespace axon

int main(int argc, char* argv[]) {
  using namespace axon::recorder;

  // Check for help flag
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
      print_usage(argv[0]);
      return 0;
    }
  }

  // Step 1: Parse command line arguments to extract config file path and CLI overrides
  std::string config_file;
  std::string cli_plugin_path;
  std::string cli_output_file;
  std::string cli_profile;
  std::string cli_compression;
  int cli_compression_level = -1;
  size_t cli_queue_capacity = 0;
  std::vector<SubscriptionConfig> cli_subscriptions;
  std::string current_topic;

  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--config") == 0) {
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
    } else if (strcmp(argv[i], "--output") == 0) {
      if (i + 1 < argc) {
        cli_output_file = argv[++i];
      } else {
        std::cerr << "Error: --output requires a file argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--topic") == 0) {
      if (i + 1 < argc) {
        current_topic = argv[++i];
      } else {
        std::cerr << "Error: --topic requires a name argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--type") == 0) {
      if (i + 1 < argc) {
        if (current_topic.empty()) {
          std::cerr << "Error: --type must follow --topic" << std::endl;
          return 1;
        }
        std::string message_type = argv[++i];
        cli_subscriptions.push_back({current_topic, message_type});
        current_topic.clear();
      } else {
        std::cerr << "Error: --type requires a type argument" << std::endl;
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
  if (!cli_output_file.empty()) {
    config.output_file = cli_output_file;
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
  if (!cli_subscriptions.empty()) {
    config.subscriptions = cli_subscriptions;
  }

  // Validate required arguments
  if (config.plugin_path.empty()) {
    std::cerr << "Error: --plugin (or plugin.path in config file) is required" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  if (config.subscriptions.empty()) {
    std::cerr << "Error: At least one --topic/--type pair is required" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  // Print configuration
  std::cout << "Axon Recorder Configuration:\n"
            << "  Plugin:      " << config.plugin_path << "\n"
            << "  Output:      " << config.output_file << "\n"
            << "  Profile:     " << config.profile << "\n"
            << "  Compression: " << config.compression << " level " << config.compression_level
            << "\n"
            << "  Queue size:  " << config.queue_capacity << "\n"
            << "  Topics:\n";
  for (const auto& sub : config.subscriptions) {
    std::cout << "    - " << sub.topic_name << " (" << sub.message_type << ")\n";
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

  // Start HTTP RPC server
  std::cout << "Starting HTTP RPC server on port 8080..." << std::endl;
  if (!recorder.start_http_server("0.0.0.0", 8080)) {
    std::cerr << "Warning: Failed to start HTTP server: " << recorder.get_last_error() << std::endl;
    std::cerr << "Continuing without HTTP RPC control..." << std::endl;
  } else {
    std::cout << "HTTP RPC server listening on http://0.0.0.0:8080" << std::endl;
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

  std::cout << "Recording saved to: " << config.output_file << std::endl;

  return 0;
}

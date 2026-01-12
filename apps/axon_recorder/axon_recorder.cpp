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

    if (g_recorder && g_recorder->is_running()) {
      g_recorder->stop();
    }
  }
}

void print_usage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [OPTIONS]\n"
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
            << "Configuration File:\n"
            << "  If --config is provided, most options can be specified in a YAML file:\n"
            << "  dataset:\n"
            << "    path: /data/recordings\n"
            << "    output_file: recording.mcap\n"
            << "    queue_size: 8192\n"
            << "  topics:\n"
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
            << "  " << program_name << " --config apps/axon_recorder/config/config.yaml \\\n"
            << "    --plugin "
               "middlewares/ros2/install/axon_ros2_plugin/lib/axon/plugins/libaxon_ros2_plugin.so\n"
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

  RecorderConfig config;
  std::string config_file;

  // Parse command line arguments
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
        config.plugin_path = argv[++i];
      } else {
        std::cerr << "Error: --plugin requires a path argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--output") == 0) {
      if (i + 1 < argc) {
        config.output_file = argv[++i];
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
        config.subscriptions.push_back({current_topic, message_type});
        current_topic.clear();
      } else {
        std::cerr << "Error: --type requires a type argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--profile") == 0) {
      if (i + 1 < argc) {
        config.profile = argv[++i];
      } else {
        std::cerr << "Error: --profile requires a profile argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--compression") == 0) {
      if (i + 1 < argc) {
        config.compression = argv[++i];
      } else {
        std::cerr << "Error: --compression requires an algorithm argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--level") == 0) {
      if (i + 1 < argc) {
        config.compression_level = std::atoi(argv[++i]);
      } else {
        std::cerr << "Error: --level requires a number argument" << std::endl;
        return 1;
      }
    } else if (strcmp(argv[i], "--queue-size") == 0) {
      if (i + 1 < argc) {
        config.queue_capacity = static_cast<size_t>(std::atol(argv[++i]));
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

  // Load configuration file if specified
  if (!config_file.empty()) {
    ConfigParser parser;
    if (!parser.load_from_file(config_file, config)) {
      std::cerr << "Error: Failed to load config file '" << config_file
                << "': " << parser.get_last_error() << std::endl;
      return 1;
    }
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

  // Start recording
  std::cout << "Starting recording..." << std::endl;
  if (!recorder.start()) {
    std::cerr << "Error: Failed to start recording: " << recorder.get_last_error() << std::endl;
    return 1;
  }

  std::cout << "Recording started. Press Ctrl+C to stop." << std::endl;

  // Monitor statistics
  uint64_t last_written = 0;
  while (!g_should_exit.load() && recorder.is_running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto stats = recorder.get_statistics();

    // Print progress every second
    if (stats.messages_written != last_written) {
      std::cout << "\rMessages: " << stats.messages_written << " written, "
                << stats.messages_received << " received, " << stats.messages_dropped
                << " dropped   " << std::flush;
      last_written = stats.messages_written;
    }
  }

  std::cout << "\nStopping recording..." << std::endl;

  // Stop recording
  recorder.stop();

  // Print final statistics
  auto final_stats = recorder.get_statistics();
  print_statistics(final_stats);

  std::cout << "Recording saved to: " << config.output_file << std::endl;

  return 0;
}

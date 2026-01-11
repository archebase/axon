// Example usage of the PluginLoader
// This demonstrates how to load a plugin from a .so file and call init, spin, shutdown

#include <chrono>
#include <iostream>
#include <thread>

#include "plugin_loader.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <plugin.so>" << std::endl;
    return 1;
  }

  std::string plugin_path = argv[1];

  // Create plugin loader
  axon::PluginLoader loader;

  // Load the plugin
  auto plugin_name = loader.load(plugin_path);
  if (!plugin_name) {
    std::cerr << "Failed to load plugin: " << loader.get_last_error() << std::endl;
    return 1;
  }

  std::cout << "Loaded plugin: " << *plugin_name << std::endl;

  // Get plugin descriptor
  const auto* descriptor = loader.get_descriptor(*plugin_name);
  if (!descriptor) {
    std::cerr << "Failed to get plugin descriptor" << std::endl;
    return 1;
  }

  // Print plugin info
  std::cout << "Middleware: "
            << (descriptor->middleware_name ? descriptor->middleware_name : "unknown") << std::endl;
  std::cout << "Version: " << (descriptor->plugin_version ? descriptor->plugin_version : "unknown")
            << std::endl;

  // Initialize the plugin with configuration (JSON string)
  const char* config_json = R"({"node_name": "axon_recorder"})";
  auto status = descriptor->vtable->init(config_json);
  if (status != axon::AXON_SUCCESS) {
    std::cerr << "Failed to initialize plugin, status: " << static_cast<int>(status) << std::endl;
    return 1;
  }
  std::cout << "Plugin initialized successfully" << std::endl;

  // Set message callback (optional)
  descriptor->vtable->set_callback(
    [](
      const char* topic_name,
      const uint8_t* message_data,
      size_t message_size,
      const char* message_type,
      uint64_t timestamp,
      void* user_data
    ) {
      std::cout << "Received message on topic: " << topic_name << ", type: " << message_type
                << ", size: " << message_size << ", timestamp: " << timestamp << std::endl;
    },
    nullptr
  );

  // Subscribe to topics (optional)
  status = descriptor->vtable->subscribe("/chatter", "std_msgs/msg/String");
  if (status != axon::AXON_SUCCESS) {
    std::cerr << "Failed to subscribe, status: " << static_cast<int>(status) << std::endl;
  }

  // Spin the plugin (this typically starts the middleware executor/loop)
  status = descriptor->vtable->spin();
  if (status != axon::AXON_SUCCESS) {
    std::cerr << "Failed to spin plugin, status: " << static_cast<int>(status) << std::endl;
    return 1;
  }
  std::cout << "Plugin spinning" << std::endl;

  // Run for a while
  std::cout << "Running for 5 seconds..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // Shutdown the plugin
  status = descriptor->vtable->shutdown();
  if (status != axon::AXON_SUCCESS) {
    std::cerr << "Failed to shutdown plugin, status: " << static_cast<int>(status) << std::endl;
    return 1;
  }
  std::cout << "Plugin shut down successfully" << std::endl;

  // Unload the plugin
  loader.unload(*plugin_name);

  return 0;
}

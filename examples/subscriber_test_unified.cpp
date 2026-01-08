#include <chrono>
#include <csignal>
#include <cstring>
#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <thread>

#include "plugin_interface.hpp"

// Global flag for shutdown
static bool g_running = true;

void signal_handler(int signal) {
  g_running = false;
}

// Global message counter for C callback
static int g_message_count = 0;

// C-style callback function (no captures allowed)
void data_callback_handler(
  const uint8_t* data, size_t size, uint64_t timestamp, const char* topic, const char* msg_type
) {
  g_message_count++;
  std::cout << "📨 [" << g_message_count << "] Received message:" << std::endl;
  std::cout << "  Topic: " << topic << std::endl;
  std::cout << "  Type: " << msg_type << std::endl;
  std::cout << "  Size: " << size << " bytes" << std::endl;
  std::cout << "  Timestamp: " << timestamp << " ns" << std::endl;

  // Show first 64 bytes of data
  size_t bytes_to_show = std::min(size_t(64), size);
  if (bytes_to_show > 0) {
    std::cout << "  Data (first " << bytes_to_show << " bytes): ";
    for (size_t i = 0; i < bytes_to_show; ++i) {
      std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    }
    std::cout << std::dec << std::endl;
  }
  std::cout << std::endl;
}

void print_usage(const char* program_name) {
  std::cout << "Usage: " << program_name << " <ros_version> [topic_name] [message_type]"
            << std::endl;
  std::cout << std::endl;
  std::cout << "Arguments:" << std::endl;
  std::cout << "  ros_version   - Either 'ros1' or 'ros2'" << std::endl;
  std::cout << "  topic_name    - ROS topic to subscribe to (default: chatter)" << std::endl;
  std::cout << "  message_type  - Message type (default: std_msgs/String for ROS1, "
               "std_msgs/msg/String for ROS2)"
            << std::endl;
  std::cout << std::endl;
  std::cout << "Examples:" << std::endl;
  std::cout << "  " << program_name << " ros2" << std::endl;
  std::cout << "  " << program_name << " ros2 /chatter std_msgs/msg/String" << std::endl;
  std::cout << "  " << program_name << " ros1 /chatter std_msgs/String" << std::endl;
}

int main(int argc, char** argv) {
  // Setup signal handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Parse command line arguments
  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }

  std::string ros_version = argv[1];
  std::string topic_name = "chatter";
  std::string message_type;

  plugin_interface::PluginInfo plugin_info;

  // Determine plugin type and set defaults
  if (ros_version == "ros1") {
    plugin_info = plugin_interface::get_ros1_plugin_info();
    message_type = "std_msgs/String";
  } else if (ros_version == "ros2") {
    plugin_info = plugin_interface::get_ros2_plugin_info();
    message_type = "std_msgs/msg/String";
  } else {
    std::cerr << "Error: Invalid ROS version '" << ros_version << "'" << std::endl;
    std::cerr << "Please use 'ros1' or 'ros2'" << std::endl;
    print_usage(argv[0]);
    return 1;
  }

  // Override defaults if provided
  if (argc > 2) {
    topic_name = argv[2];
  }
  if (argc > 3) {
    message_type = argv[3];
  }

  std::cout << "=== " << plugin_info.name << " Plugin Loader ===" << std::endl;
  std::cout << "Loading plugin library: " << plugin_info.library_name << std::endl;

  // Load the shared library
  void* handle = dlopen(plugin_info.library_name, RTLD_LAZY);
  if (!handle) {
    std::cerr << "Cannot open library: " << dlerror() << std::endl;
    std::cerr << std::endl;
    std::cerr << "Make sure the plugin library is built and available:" << std::endl;
    if (plugin_info.type == plugin_interface::PluginType::ROS1) {
      std::cerr << "  cd ros1 && ./build.sh" << std::endl;
    } else {
      std::cerr << "  ./build.sh" << std::endl;
    }
    return 1;
  }

  std::cout << "✓ Library loaded successfully!" << std::endl;

  // Load initialization functions
  plugin_interface::InitContextFunc init_context =
    (plugin_interface::InitContextFunc)dlsym(handle, plugin_info.init_func_name);
  plugin_interface::ShutdownContextFunc shutdown_context =
    (plugin_interface::ShutdownContextFunc)dlsym(handle, plugin_info.shutdown_func_name);

  if (!init_context) {
    std::cerr << "Failed to load " << plugin_info.init_func_name << " function: " << dlerror()
              << std::endl;
    dlclose(handle);
    return 1;
  }

  // Initialize ROS context
  std::cout << "Initializing " << plugin_info.name << "..." << std::endl;
  init_context();
  std::cout << "✓ " << plugin_info.name << " initialized successfully!" << std::endl;

  // Load factory functions
  plugin_interface::CreateSubscriberFunc create_subscriber =
    (plugin_interface::CreateSubscriberFunc)dlsym(handle, "create_subscriber");
  plugin_interface::DestroySubscriberFunc destroy_subscriber =
    (plugin_interface::DestroySubscriberFunc)dlsym(handle, "destroy_subscriber");
  plugin_interface::SpinSubscriberFunc spin_subscriber =
    (plugin_interface::SpinSubscriberFunc)dlsym(handle, plugin_info.spin_func_name);
  plugin_interface::GetTopicNameFunc get_topic_name =
    (plugin_interface::GetTopicNameFunc)dlsym(handle, "get_topic_name");
  plugin_interface::GetMessageTypeFunc get_message_type =
    (plugin_interface::GetMessageTypeFunc)dlsym(handle, "get_message_type");
  plugin_interface::SetDataCallbackFunc set_data_callback =
    (plugin_interface::SetDataCallbackFunc)dlsym(handle, "set_data_callback");

  if (!create_subscriber || !destroy_subscriber || !spin_subscriber) {
    std::cerr << "Failed to load factory functions: " << dlerror() << std::endl;
    dlclose(handle);
    return 1;
  }

  // Create subscriber
  std::cout << "Creating subscriber for topic '" << topic_name << "' with message type '"
            << message_type << "'" << std::endl;
  void* subscriber = create_subscriber(topic_name.c_str(), message_type.c_str());

  if (!subscriber) {
    std::cerr << "Failed to create subscriber" << std::endl;
    if (shutdown_context) {
      shutdown_context();
    }
    dlclose(handle);
    return 1;
  }

  if (get_topic_name) {
    std::cout << "✓ Subscriber created for topic: " << get_topic_name(subscriber) << std::endl;
  }

  if (get_message_type) {
    std::cout << "  Message type: " << get_message_type(subscriber) << std::endl;
  }

  // Set data callback
  if (set_data_callback) {
    set_data_callback(subscriber, data_callback_handler);
    std::cout << "✓ Data callback registered" << std::endl;
  }

  std::cout << std::endl;
  std::cout << "=== Listening for messages (Ctrl+C to exit) ===" << std::endl;
  std::cout << std::endl;

  if (plugin_info.type == plugin_interface::PluginType::ROS1) {
    std::cout << "Note: Make sure ROS1 is running (roscore) and a publisher is active."
              << std::endl;
    std::cout << std::endl;
    std::cout << "Try running:" << std::endl;
    std::cout << "  rostopic pub " << topic_name << " " << message_type << " \"data: 'Hello'\" -r 1"
              << std::endl;
  } else {
    std::cout << "Note: Make sure ROS2 is sourced and a publisher is active." << std::endl;
    std::cout << std::endl;
    std::cout << "Try running:" << std::endl;
    std::cout << "  ros2 topic pub " << topic_name << " " << message_type
              << " \"data: 'Hello World'\"" << std::endl;
  }
  std::cout << std::endl;

  // Spin loop
  while (g_running) {
    spin_subscriber(subscriber);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << std::endl;
  std::cout << "Shutting down..." << std::endl;

  // Cleanup
  if (set_data_callback) {
    set_data_callback(subscriber, nullptr);
  }

  if (destroy_subscriber) {
    destroy_subscriber(subscriber);
  }

  if (shutdown_context) {
    shutdown_context();
  }

  dlclose(handle);
  std::cout << "✓ Plugin unloaded successfully" << std::endl;
  std::cout << "✓ Total messages received: " << g_message_count << std::endl;

  return 0;
}

/**
 * Axon Recorder - Main Entry Point
 *
 * This example shows how to load the ROS service adapter plugin dynamically
 * and use it to provide recording services.
 *
 * The core recorder has NO compile-time dependencies on ROS1 or ROS2.
 * Everything is loaded at runtime via plugins.
 */

#include <axon_utils/common_types.hpp>
#include <axon_utils/recorder_service_interface.hpp>
#include <axon_utils/state_machine.hpp>

#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

// Example: Simple recorder context implementation
// In the full implementation, this would interface with RecordingSession, TopicManager, etc.
class ExampleRecorderContext : public axon::utils::IRecorderContext {
public:
  ExampleRecorderContext() {
    // Initialize state machine with recorder states
    std::unordered_map<RecorderState, std::vector<RecorderState>> transitions = {
      {RecorderState::IDLE, {RecorderState::READY}},
      {RecorderState::READY, {RecorderState::IDLE, RecorderState::RECORDING}},
      {RecorderState::RECORDING, {RecorderState::PAUSED, RecorderState::IDLE}},
      {RecorderState::PAUSED, {RecorderState::RECORDING, RecorderState::IDLE}}};

    state_machine_ = std::make_unique<StateMachineType>(RecorderState::IDLE, transitions);
  }

  std::string get_state_string() const override {
    switch (state_machine_->get_state()) {
      case RecorderState::IDLE:
        return "idle";
      case RecorderState::READY:
        return "ready";
      case RecorderState::RECORDING:
        return "recording";
      case RecorderState::PAUSED:
        return "paused";
      default:
        return "unknown";
    }
  }

  bool is_recording_active() const override {
    return state_machine_->is_state_in({RecorderState::RECORDING, RecorderState::PAUSED});
  }

  std::optional<axon::utils::TaskConfig> get_cached_config() const override {
    return task_cache_.get();
  }

  bool get_recording_metrics(
    std::string& output_path, double& disk_usage_gb, double& duration_sec, int64_t& message_count,
    double& throughput_mb_sec, std::string& last_error
  ) const override {
    // TODO: Implement actual metrics gathering
    output_path = "/tmp/example.mcap";
    disk_usage_gb = 0.5;
    duration_sec = 10.0;
    message_count = 1000;
    throughput_mb_sec = 1.5;
    last_error.clear();
    return true;
  }

  bool cache_task_config(const axon::utils::TaskConfig& config, std::string& error_msg) override {
    std::string error;
    if (!state_machine_->transition(RecorderState::IDLE, RecorderState::READY, error)) {
      error_msg = "State transition failed: " + error;
      return false;
    }

    task_cache_.cache(config);
    return true;
  }

  bool start_recording(std::string& error_msg) override {
    std::string error;
    return state_machine_->transition(RecorderState::READY, RecorderState::RECORDING, error);
  }

  bool pause_recording(std::string& error_msg) override {
    std::string error;
    return state_machine_->transition(RecorderState::RECORDING, RecorderState::PAUSED, error);
  }

  bool resume_recording(std::string& error_msg) override {
    std::string error;
    return state_machine_->transition(RecorderState::PAUSED, RecorderState::RECORDING, error);
  }

  bool cancel_recording(std::string& error_msg) override {
    std::string error;
    if (state_machine_->transition(RecorderState::RECORDING, RecorderState::IDLE, error) ||
        state_machine_->transition(RecorderState::PAUSED, RecorderState::IDLE, error)) {
      task_cache_.clear();
      return true;
    }
    error_msg = error;
    return false;
  }

  bool finish_recording(std::string& error_msg) override {
    std::string error;
    if (state_machine_->transition(RecorderState::RECORDING, RecorderState::IDLE, error) ||
        state_machine_->transition(RecorderState::PAUSED, RecorderState::IDLE, error)) {
      task_cache_.clear();
      return true;
    }
    error_msg = error;
    return false;
  }

  bool clear_config(std::string& error_msg) override {
    std::string error;
    if (state_machine_->transition(RecorderState::READY, RecorderState::IDLE, error)) {
      task_cache_.clear();
      return true;
    }
    error_msg = error;
    return false;
  }

private:
  enum class RecorderState { IDLE, READY, RECORDING, PAUSED };

  using StateMachineType = axon::utils::StateMachine<RecorderState>;

  std::unique_ptr<StateMachineType> state_machine_;
  axon::utils::TaskConfigCache task_cache_;
};

/**
 * Load ROS2 service adapter plugin
 */
std::unique_ptr<axon::utils::IRecorderServiceAdapter> load_ros2_plugin(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node
) {
  // Path to the plugin library (adjust as needed)
  const char* plugin_path = "middlewares/ros2/install/ros2_plugin/lib/libros2_plugin_lib.so";

  // Load the plugin
  void* handle = dlopen(plugin_path, RTLD_LAZY);
  if (!handle) {
    std::cerr << "Failed to load plugin: " << dlerror() << std::endl;
    return nullptr;
  }

  // Get the factory function
  using CreateFunc =
    ros2_plugin::ServiceAdapter* (*)(std::shared_ptr<axon::utils::IRecorderContext>, void*);

  auto create_func =
    reinterpret_cast<CreateFunc>(dlsym(handle, "create_recorder_service_adapter_ros2"));
  if (!create_func) {
    std::cerr << "Failed to find factory function: " << dlerror() << std::endl;
    dlclose(handle);
    return nullptr;
  }

  // Create the adapter
  auto* adapter_ptr = create_func(context, ros_node);
  if (!adapter_ptr) {
    std::cerr << "Failed to create adapter instance" << std::endl;
    dlclose(handle);
    return nullptr;
  }

  // Wrap in unique_ptr with custom deleter
  return std::unique_ptr<axon::utils::IRecorderServiceAdapter>(adapter_ptr);
}

/**
 * Load ROS1 service adapter plugin
 */
std::unique_ptr<axon::utils::IRecorderServiceAdapter> load_ros1_plugin(
  std::shared_ptr<axon::utils::IRecorderContext> context, void* ros_node_handle
) {
  // Path to the plugin library (adjust as needed)
  const char* plugin_path =
    "middlewares/ros1/build/libaxon_recorder_plugin/libaxon_recorder_plugin.so";

  // Load the plugin
  void* handle = dlopen(plugin_path, RTLD_LAZY);
  if (!handle) {
    std::cerr << "Failed to load plugin: " << dlerror() << std::endl;
    return nullptr;
  }

  // Get the factory function
  using CreateFunc = axon_recorder_plugin::
    RecorderServiceAdapterROS1* (*)(std::shared_ptr<axon::utils::IRecorderContext>, void*);

  auto create_func =
    reinterpret_cast<CreateFunc>(dlsym(handle, "create_recorder_service_adapter_ros1"));
  if (!create_func) {
    std::cerr << "Failed to find factory function: " << dlerror() << std::endl;
    dlclose(handle);
    return nullptr;
  }

  // Create the adapter
  auto* adapter_ptr = create_func(context, ros_node_handle);
  if (!adapter_ptr) {
    std::cerr << "Failed to create adapter instance" << std::endl;
    dlclose(handle);
    return nullptr;
  }

  // Wrap in unique_ptr with custom deleter
  return std::unique_ptr<axon::utils::IRecorderServiceAdapter>(adapter_ptr);
}

int main(int argc, char** argv) {
  std::cout << "Axon Recorder Example" << std::endl;
  std::cout << "=====================" << std::endl;

  // Create the recorder context (implements IRecorderContext)
  auto context = std::make_shared<ExampleRecorderContext>();

  // TODO: Initialize ROS node
  // For ROS2:
  //   rclcpp::init(argc, argv);
  //   auto node = std::make_shared<rclcpp::Node>("axon_recorder");
  //
  // For ROS1:
  //   ros::init(argc, argv, "axon_recorder");
  //   ros::NodeHandle nh;

  // Load appropriate plugin based on ROS version
  std::unique_ptr<axon::utils::IRecorderServiceAdapter> service_adapter;

  // TODO: Detect ROS version and load appropriate plugin
  // For now, this is just a skeleton

  std::cout << "NOTE: This is a skeleton example." << std::endl;
  std::cout << "The full implementation would:" << std::endl;
  std::cout << "  1. Initialize ROS node (ROS1 or ROS2)" << std::endl;
  std::cout << "  2. Load the appropriate plugin" << std::endl;
  std::cout << "  3. Register services" << std::endl;
  std::cout << "  4. Spin until shutdown" << std::endl;
  std::cout << std::endl;
  std::cout << "See MIGRATION_SUMMARY.md for details." << std::endl;

  // In the full implementation:
  // service_adapter->register_services();
  // ... spin ...
  // service_adapter->shutdown();

  return 0;
}

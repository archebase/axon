#ifndef AXON_MESSAGE_REGISTRY_HPP
#define AXON_MESSAGE_REGISTRY_HPP

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace axon {
namespace recorder {

/**
 * Abstract message converter interface (simplified for MCAP backend)
 *
 * With MCAP, raw serialized messages are written directly, so converters
 * are optional and only needed for advanced use cases like transcoding.
 */
class MessageConverter {
public:
  virtual ~MessageConverter() = default;
  virtual std::string get_message_type() const = 0;
};

/**
 * Registry for ROS message type handlers
 * Maps message types to conversion functions
 *
 * Note: With MCAP backend, converters are optional since raw serialized
 * messages are written directly to the file.
 */
class MessageRegistry {
public:
  using ConverterFactory = std::function<std::unique_ptr<MessageConverter>()>;

  /**
   * Register a message type converter
   */
  static void register_message_type(const std::string& message_type, ConverterFactory factory);

  /**
   * Get converter for a message type
   */
  static std::unique_ptr<MessageConverter> get_converter(const std::string& message_type);

  /**
   * Check if a message type is registered
   */
  static bool has_converter(const std::string& message_type);

  /**
   * Get all registered message types
   */
  static std::vector<std::string> get_registered_types();

  /**
   * Initialize default converters for common ROS message types
   */
  static void initialize_default_converters();

private:
  static std::unordered_map<std::string, ConverterFactory>& get_registry();
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_MESSAGE_REGISTRY_HPP

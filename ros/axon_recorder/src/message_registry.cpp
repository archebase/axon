#include "message_registry.hpp"

#include <iostream>

namespace axon {
namespace recorder {

static std::unordered_map<std::string, MessageRegistry::ConverterFactory>* g_registry = nullptr;

std::unordered_map<std::string, MessageRegistry::ConverterFactory>&
MessageRegistry::get_registry() {
  if (!g_registry) {
    g_registry = new std::unordered_map<std::string, MessageRegistry::ConverterFactory>();
  }
  return *g_registry;
}

void MessageRegistry::register_message_type(
  const std::string& message_type, ConverterFactory factory
) {
  auto& registry = get_registry();
  registry[message_type] = factory;
}

std::unique_ptr<MessageConverter> MessageRegistry::get_converter(const std::string& message_type) {
  auto& registry = get_registry();
  auto it = registry.find(message_type);
  if (it != registry.end()) {
    return it->second();
  }
  return nullptr;
}

bool MessageRegistry::has_converter(const std::string& message_type) {
  auto& registry = get_registry();
  return registry.find(message_type) != registry.end();
}

std::vector<std::string> MessageRegistry::get_registered_types() {
  auto& registry = get_registry();
  std::vector<std::string> types;
  types.reserve(registry.size());
  for (const auto& pair : registry) {
    types.push_back(pair.first);
  }
  return types;
}

void MessageRegistry::initialize_default_converters() {
  // With MCAP backend, converters are not needed for recording
  // Raw serialized messages are written directly to MCAP
  std::cout << "MessageRegistry: MCAP backend - converters optional" << std::endl;
}

}  // namespace recorder
}  // namespace axon

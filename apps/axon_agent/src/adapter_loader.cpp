// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "adapter_loader.hpp"

#include <dlfcn.h>
#include <exception>
#include <sstream>

namespace axon {
namespace agent {

AdapterLoader::~AdapterLoader() {
  unload();
}

bool AdapterLoader::load(const RobotProfile& profile, std::string* error) {
  if (is_loaded() && loaded_profile_id_ == profile.profile_id) {
    return true;
  }

  unload();

  if (profile.library_path.empty()) {
    if (error != nullptr) {
      *error = "profile does not declare adapter library: " + profile.profile_id;
    }
    return false;
  }

  handle_ = dlopen(profile.library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (handle_ == nullptr) {
    if (error != nullptr) {
      *error = dlerror();
    }
    return false;
  }

  dlerror();
  get_descriptor_fn_ =
    reinterpret_cast<GetRobotAdapterDescriptorFn>(dlsym(handle_, kRobotAdapterDescriptorSymbol));
  const char* descriptor_error = dlerror();
  if (descriptor_error != nullptr) {
    unload();
    if (error != nullptr) {
      *error = "adapter descriptor symbol not found: " + std::string(kRobotAdapterDescriptorSymbol);
    }
    return false;
  }

  dlerror();
  create_fn_ = reinterpret_cast<CreateRobotAdapterFn>(dlsym(handle_, profile.entry_symbol.c_str()));
  const char* symbol_error = dlerror();
  if (symbol_error != nullptr) {
    unload();
    if (error != nullptr) {
      *error = "adapter entry symbol not found: " + profile.entry_symbol;
    }
    return false;
  }

  dlerror();
  destroy_fn_ = reinterpret_cast<DestroyRobotAdapterFn>(dlsym(handle_, kRobotAdapterDestroySymbol));
  const char* destroy_error = dlerror();
  if (destroy_error != nullptr) {
    unload();
    if (error != nullptr) {
      *error = "adapter destroy symbol not found: " + std::string(kRobotAdapterDestroySymbol);
    }
    return false;
  }

  const auto* descriptor = get_descriptor_fn_();
  if (!validate_descriptor(profile, descriptor, error)) {
    unload();
    return false;
  }
  descriptor_ = *descriptor;

  try {
    adapter_ = create_fn_();
  } catch (const std::exception& ex) {
    unload();
    if (error != nullptr) {
      *error = "adapter create threw exception: " + std::string(ex.what());
    }
    return false;
  } catch (...) {
    unload();
    if (error != nullptr) {
      *error = "adapter create threw unknown exception";
    }
    return false;
  }

  if (adapter_ == nullptr) {
    unload();
    if (error != nullptr) {
      *error = "adapter create returned null";
    }
    return false;
  }

  loaded_profile_id_ = profile.profile_id;
  loaded_library_ = profile.library_path;
  return true;
}

void AdapterLoader::unload() {
  if (adapter_ != nullptr && destroy_fn_ != nullptr) {
    try {
      destroy_fn_(adapter_);
    } catch (...) {
      // Adapter destroy must not throw across the plugin boundary.
    }
  }
  adapter_ = nullptr;
  get_descriptor_fn_ = nullptr;
  create_fn_ = nullptr;
  destroy_fn_ = nullptr;
  descriptor_ = RobotAdapterDescriptor();

  if (handle_ != nullptr) {
    dlclose(handle_);
    handle_ = nullptr;
  }
  loaded_profile_id_.clear();
  loaded_library_.clear();
}

bool AdapterLoader::is_loaded() const {
  return handle_ != nullptr && adapter_ != nullptr;
}

bool AdapterLoader::is_loaded_for_profile(const std::string& profile_id) const {
  return is_loaded() && loaded_profile_id_ == profile_id;
}

nlohmann::json AdapterLoader::status_to_json() const {
  nlohmann::json status = {
    {"loaded", is_loaded()},
    {"profile_id", loaded_profile_id_},
    {"library", loaded_library_.string()},
  };
  if (is_loaded()) {
    status["descriptor"] = descriptor_to_json(descriptor_);
  }
  return status;
}

nlohmann::json AdapterLoader::runtime_status_to_json(const RobotAdapterContext& context) const {
  if (!is_loaded()) {
    return nullptr;
  }
  try {
    return status_to_json(adapter_->status(context));
  } catch (const std::exception& ex) {
    return {{"reachable", false}, {"state", "error"}, {"message", ex.what()}};
  } catch (...) {
    return {
      {"reachable", false},
      {"state", "error"},
      {"message", "adapter status threw unknown exception"}
    };
  }
}

bool AdapterLoader::start(const RobotAdapterContext& context, std::string* error) {
  if (!is_loaded()) {
    if (error != nullptr) {
      *error = "adapter is not loaded";
    }
    return false;
  }
  try {
    const auto result = adapter_->start(context);
    if (!result.success && error != nullptr) {
      *error = result.message;
    }
    return result.success;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "adapter start threw unknown exception";
    }
    return false;
  }
}

bool AdapterLoader::stop(const RobotAdapterContext& context, int timeout_ms, std::string* error) {
  if (!is_loaded()) {
    if (error != nullptr) {
      *error = "adapter is not loaded";
    }
    return false;
  }
  try {
    const auto result = adapter_->stop(context, timeout_ms);
    if (!result.success && error != nullptr) {
      *error = result.message;
    }
    return result.success;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "adapter stop threw unknown exception";
    }
    return false;
  }
}

bool AdapterLoader::force_stop(const RobotAdapterContext& context, std::string* error) {
  if (!is_loaded()) {
    if (error != nullptr) {
      *error = "adapter is not loaded";
    }
    return false;
  }
  try {
    const auto result = adapter_->force_stop(context);
    if (!result.success && error != nullptr) {
      *error = result.message;
    }
    return result.success;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  } catch (...) {
    if (error != nullptr) {
      *error = "adapter force_stop threw unknown exception";
    }
    return false;
  }
}

bool AdapterLoader::validate_descriptor(
  const RobotProfile& profile, const RobotAdapterDescriptor* descriptor, std::string* error
) {
  if (descriptor == nullptr) {
    if (error != nullptr) {
      *error = "adapter descriptor is null";
    }
    return false;
  }
  if (descriptor->abi_version != profile.abi_version) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << "adapter ABI mismatch: profile expects " << profile.abi_version
             << ", plugin exports " << descriptor->abi_version;
      *error = stream.str();
    }
    return false;
  }
  if (descriptor->abi_version != kRobotAdapterAbiVersion) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << "unsupported robot adapter ABI: " << descriptor->abi_version;
      *error = stream.str();
    }
    return false;
  }
  if (descriptor->adapter_id == nullptr || std::string(descriptor->adapter_id).empty()) {
    if (error != nullptr) {
      *error = "adapter descriptor adapter_id is empty";
    }
    return false;
  }
  if (profile.adapter_id != descriptor->adapter_id) {
    if (error != nullptr) {
      *error = "adapter id mismatch: profile expects " + profile.adapter_id + ", plugin exports " +
               std::string(descriptor->adapter_id);
    }
    return false;
  }
  return true;
}

nlohmann::json AdapterLoader::descriptor_to_json(const RobotAdapterDescriptor& descriptor) {
  return {
    {"abi_version", descriptor.abi_version},
    {"adapter_id", descriptor.adapter_id == nullptr ? "" : descriptor.adapter_id},
    {"robot_model", descriptor.robot_model == nullptr ? "" : descriptor.robot_model},
    {"adapter_version", descriptor.adapter_version == nullptr ? "" : descriptor.adapter_version},
    {"vendor", descriptor.vendor == nullptr ? "" : descriptor.vendor},
  };
}

nlohmann::json AdapterLoader::status_to_json(const RobotAdapterStatus& status) {
  return {
    {"reachable", status.reachable},
    {"state", status.state},
    {"message", status.message},
    {"log_path", status.log_path},
  };
}

}  // namespace agent
}  // namespace axon

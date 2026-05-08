// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "adapter_loader.hpp"

#include <dlfcn.h>

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
  (void)dlsym(handle_, profile.entry_symbol.c_str());
  const char* symbol_error = dlerror();
  if (symbol_error != nullptr) {
    unload();
    if (error != nullptr) {
      *error = "adapter entry symbol not found: " + profile.entry_symbol;
    }
    return false;
  }

  loaded_profile_id_ = profile.profile_id;
  loaded_library_ = profile.library_path;
  return true;
}

void AdapterLoader::unload() {
  if (handle_ != nullptr) {
    dlclose(handle_);
    handle_ = nullptr;
  }
  loaded_profile_id_.clear();
  loaded_library_.clear();
}

bool AdapterLoader::is_loaded() const {
  return handle_ != nullptr;
}

nlohmann::json AdapterLoader::status_to_json() const {
  return {
    {"loaded", is_loaded()},
    {"profile_id", loaded_profile_id_},
    {"library", loaded_library_.string()},
  };
}

}  // namespace agent
}  // namespace axon

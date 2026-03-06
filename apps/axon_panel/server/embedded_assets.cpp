/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#include "embedded_assets.hpp"

std::unordered_map<std::string, Asset> EmbeddedAssets::assets_;
bool EmbeddedAssets::initialized_ = false;

const Asset* EmbeddedAssets::get(const std::string& path) {
  if (!initialized_) {
    init();
  }

  auto it = assets_.find(path);
  if (it != assets_.end()) {
    return &it->second;
  }

  return nullptr;
}

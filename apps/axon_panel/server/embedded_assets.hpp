/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 *
 * SPDX-License-Identifier: MulanPSL-2.0
 */

#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>

struct Asset {
  const char* data;
  size_t size;
  std::string mime_type;
};

class EmbeddedAssets {
public:
  static const Asset* get(const std::string& path);
  static void init();

private:
  static std::unordered_map<std::string, Asset> assets_;
  static bool initialized_;
};

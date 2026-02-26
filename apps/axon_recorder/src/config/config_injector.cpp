// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "config_injector.hpp"

#include <mcap/mcap.hpp>
#include <mcap/reader.hpp>

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <system_error>

#include "mcap_writer_wrapper.hpp"

namespace fs = std::filesystem;

namespace axon {
namespace recorder {

namespace {

// Cache file name (must match axon_config)
constexpr const char* CACHE_FILENAME = "cache.mcap";

// Enabled marker file name (must match axon_config)
constexpr const char* ENABLED_MARKER = ".enabled";

}  // namespace

ConfigInjector::ConfigInjector() = default;

ConfigInjector::~ConfigInjector() = default;

std::string ConfigInjector::enabled_marker_path() const {
  // Get user's home directory from environment
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.axon/" + ENABLED_MARKER;
  }
  // Fallback for testing or non-standard environments
  return "/tmp/axon/" + std::string(ENABLED_MARKER);
}

std::string ConfigInjector::cache_path() const {
  // Get user's home directory from environment
  const char* home = std::getenv("HOME");
  if (home && home[0] != '\0') {
    return std::string(home) + "/.axon/" + CACHE_FILENAME;
  }
  // Fallback for testing or non-standard environments
  return "/tmp/axon/" + std::string(CACHE_FILENAME);
}

bool ConfigInjector::is_enabled() const {
  std::string marker_path = enabled_marker_path();
  std::error_code ec;
  bool exists = fs::exists(fs::path(marker_path), ec);
  return exists && !ec;
}

bool ConfigInjector::read_cache_attachments(
  std::vector<CachedAttachment>& attachments, std::string& error_msg
) {
  std::string cache_file = cache_path();

  // Check if cache exists
  std::error_code ec;
  if (!fs::exists(fs::path(cache_file), ec)) {
    error_msg = "Config cache not found: " + cache_file;
    return false;
  }

  // Open MCAP file
  mcap::McapReader reader;
  mcap::Status status = reader.open(cache_file);
  if (!status.ok()) {
    error_msg = "Failed to open cache file: " + std::string(status.message);
    return false;
  }

  // Read summary to populate indexes
  status = reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan);
  if (!status.ok()) {
    reader.close();
    error_msg = "Failed to read cache summary: " + std::string(status.message);
    return false;
  }

  // Get attachment indexes
  const auto& indexes = reader.attachmentIndexes();
  attachments.clear();
  attachments.reserve(indexes.size());

  // Get data source for reading raw records
  mcap::IReadable* data_source = reader.dataSource();
  if (!data_source) {
    reader.close();
    error_msg = "Failed to get data source";
    return false;
  }

  // Read each attachment's data using ReadRecord
  for (const auto& [name, index] : indexes) {
    // Read the raw record at the attachment's offset
    mcap::Record record;
    status = mcap::McapReader::ReadRecord(*data_source, index.offset, &record);
    if (!status.ok()) {
      reader.close();
      error_msg = "Failed to read attachment record: " + name + " - " + std::string(status.message);
      return false;
    }

    // Parse the attachment
    mcap::Attachment attachment;
    status = mcap::McapReader::ParseAttachment(record, &attachment);
    if (!status.ok()) {
      reader.close();
      error_msg = "Failed to parse attachment: " + name + " - " + std::string(status.message);
      return false;
    }

    // Create cached attachment
    CachedAttachment cached;
    cached.name = attachment.name;
    cached.content_type = attachment.mediaType;

    // Copy attachment data (std::byte to uint8_t)
    cached.data.resize(attachment.dataSize);
    std::memcpy(cached.data.data(), attachment.data, attachment.dataSize);

    attachments.push_back(std::move(cached));
  }

  reader.close();
  return true;
}

// Template method implementations

template<typename WriterType>
ConfigInjector::InjectionResult ConfigInjector::inject(WriterType& writer) {
  InjectionResult result;
  result.cache_path = cache_path();

  // Check if enabled
  if (!is_enabled()) {
    result.success = true;  // Not an error, just disabled
    result.error_message = "Config injection disabled (no .enabled marker)";
    return result;
  }

  // Read cache and inject attachments
  if (!do_inject(writer, result)) {
    result.success = false;
    return result;
  }

  result.success = true;
  return result;
}

template<typename WriterType>
bool ConfigInjector::do_inject(WriterType& writer, InjectionResult& result) {
  // Read all attachments from cache
  std::vector<CachedAttachment> attachments;
  if (!read_cache_attachments(attachments, result.error_message)) {
    return false;
  }

  if (attachments.empty()) {
    result.error_message = "Cache exists but has no attachments";
    return true;  // Not a critical error
  }

  // Write each attachment to the target MCAP with "config/" prefix
  for (const auto& attachment : attachments) {
    std::string target_name = "config/" + attachment.name;

    if (!writer.write_attachment(
          target_name, attachment.content_type, attachment.data.data(), attachment.data.size()
        )) {
      result.error_message = "Failed to write attachment: " + target_name;
      return false;
    }

    result.files_injected++;
    result.total_bytes += attachment.data.size();
  }

  return true;
}

// Explicit template instantiation for McapWriterWrapper
template ConfigInjector::InjectionResult ConfigInjector::inject<mcap_wrapper::McapWriterWrapper>(
  mcap_wrapper::McapWriterWrapper&
);
template bool ConfigInjector::do_inject<mcap_wrapper::McapWriterWrapper>(
  mcap_wrapper::McapWriterWrapper&, InjectionResult&
);

}  // namespace recorder
}  // namespace axon

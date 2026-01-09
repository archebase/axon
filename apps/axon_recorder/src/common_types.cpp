#include "common_types.hpp"

#include <sstream>

namespace axon {
namespace recorder {

bool RecorderConfig::validate() const {
  // Check if dataset path is set
  if (dataset.path.empty()) {
    return false;
  }

  // Check if at least one topic is configured
  if (topics.empty()) {
    return false;
  }

  // Validate each topic
  for (const auto& topic : topics) {
    if (topic.name.empty()) {
      return false;
    }
    if (topic.message_type.empty()) {
      return false;
    }
  }

  // Validate upload config if enabled
  if (upload.enabled) {
    if (upload.s3.bucket.empty()) {
      return false;
    }
  }

  return true;
}

std::string RecorderConfig::to_string() const {
  std::ostringstream oss;
  oss << "RecorderConfig:\n";
  oss << "  Dataset Path: " << dataset.path << "\n";
  oss << "  Mode: " << dataset.mode << "\n";
  oss << "  Stats File: " << dataset.stats_file_path << "\n";
  oss << "  Topics (" << topics.size() << "):\n";

  for (const auto& topic : topics) {
    oss << "    - " << topic.name << " [" << topic.message_type << "] "
        << "(batch=" << topic.batch_size << ", flush=" << topic.flush_interval_ms << "ms)\n";
  }

  oss << "  Max Disk Usage: " << recording.max_disk_usage_gb << " GB\n";
  oss << "  Upload Enabled: " << (upload.enabled ? "Yes" : "No") << "\n";

  if (upload.enabled) {
    oss << "    S3 Bucket: " << upload.s3.bucket << "\n";
    oss << "    S3 Region: " << upload.s3.region << "\n";
    oss << "    Workers: " << upload.num_workers << "\n";
  }

  return oss.str();
}

}  // namespace recorder
}  // namespace axon

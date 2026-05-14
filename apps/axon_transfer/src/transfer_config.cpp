// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "transfer_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <iostream>

namespace axon {
namespace transfer {

namespace {

std::string get_env(const char* name, const std::string& default_val) {
  const char* val = std::getenv(name);
  return (val && val[0] != '\0') ? std::string(val) : default_val;
}

std::chrono::milliseconds get_env_ms(const char* name, std::chrono::milliseconds default_val) {
  const char* val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  try {
    return std::chrono::milliseconds(std::stoll(val));
  } catch (...) {
    return default_val;
  }
}

bool get_env_bool(const char* name, bool default_val) {
  const char* val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  return std::string(val) == "true" || std::string(val) == "1" || std::string(val) == "yes";
}

}  // namespace

TransferConfig load_config(const std::string& yaml_path) {
  TransferConfig config;

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load config file " << yaml_path << ": " << e.what() << "\n";
    throw;
  }

  config.device_id = get_env("AXON_DEVICE_ID", yaml["device_id"].as<std::string>(""));
  config.factory_id = get_env("AXON_FACTORY_ID", yaml["factory_id"].as<std::string>(""));

  config.ws.url = get_env("AXON_TRANSFER_WS_URL", config.ws.url);

  if (yaml["ws"]) {
    config.ws.url =
      get_env("AXON_TRANSFER_WS_URL", yaml["ws"]["url"].as<std::string>(config.ws.url));
    config.ws.ping_interval_ms =
      std::chrono::milliseconds(yaml["ws"]["ping_interval_ms"].as<int>(30000));
    config.ws.ping_timeout_ms =
      std::chrono::milliseconds(yaml["ws"]["ping_timeout_ms"].as<int>(10000));

    if (yaml["ws"]["reconnect"]) {
      config.ws.reconnect.initial_delay_ms =
        std::chrono::milliseconds(yaml["ws"]["reconnect"]["initial_delay_ms"].as<int>(1000));
      config.ws.reconnect.backoff_multiplier =
        yaml["ws"]["reconnect"]["backoff_multiplier"].as<double>(2.0);
      config.ws.reconnect.max_delay_ms =
        std::chrono::milliseconds(yaml["ws"]["reconnect"]["max_delay_ms"].as<int>(60000));
      config.ws.reconnect.jitter_factor = yaml["ws"]["reconnect"]["jitter_factor"].as<double>(0.2);
    }
  }

  if (yaml["scanner"]) {
    config.scanner.data_dir = get_env(
      "AXON_TRANSFER_DATA_DIR", yaml["scanner"]["data_dir"].as<std::string>(config.scanner.data_dir)
    );
    config.scanner.require_json_sidecar =
      yaml["scanner"]["require_json_sidecar"].as<bool>(config.scanner.require_json_sidecar);
    config.scanner.completion_marker_suffix =
      yaml["scanner"]["completion_marker_suffix"].as<std::string>(
        config.scanner.completion_marker_suffix
      );
    config.scanner.min_ready_age_ms = std::chrono::milliseconds(
      yaml["scanner"]["min_ready_age_ms"].as<int>(config.scanner.min_ready_age_ms.count())
    );
  } else {
    config.scanner.data_dir = get_env("AXON_TRANSFER_DATA_DIR", config.scanner.data_dir);
  }

  if (yaml["uploader"]) {
    config.uploader.state_db_path =
      yaml["uploader"]["state_db_path"].as<std::string>(config.uploader.state_db_path);
    config.uploader.delete_after_upload = get_env_bool(
      "AXON_TRANSFER_DELETE_AFTER_UPLOAD", yaml["uploader"]["delete_after_upload"].as<bool>(true)
    );
    config.uploader.failed_uploads_dir =
      yaml["uploader"]["failed_uploads_dir"].as<std::string>(config.uploader.failed_uploads_dir);
    config.uploader.num_workers = yaml["uploader"]["num_workers"].as<int>(2);

    if (yaml["uploader"]["s3"]) {
      config.uploader.s3.endpoint_url =
        get_env("AXON_S3_ENDPOINT", yaml["uploader"]["s3"]["endpoint_url"].as<std::string>(""));
      config.uploader.s3.bucket =
        get_env("AXON_S3_BUCKET", yaml["uploader"]["s3"]["bucket"].as<std::string>(""));
      config.uploader.s3.region =
        yaml["uploader"]["s3"]["region"].as<std::string>("cn-northwest-1");
      config.uploader.s3.access_key =
        get_env("AWS_ACCESS_KEY_ID", yaml["uploader"]["s3"]["access_key"].as<std::string>(""));
      config.uploader.s3.secret_key =
        get_env("AWS_SECRET_ACCESS_KEY", yaml["uploader"]["s3"]["secret_key"].as<std::string>(""));
      config.uploader.s3.part_size = yaml["uploader"]["s3"]["part_size"].as<uint64_t>(67108864);
      config.uploader.s3.connect_timeout_ms =
        yaml["uploader"]["s3"]["connect_timeout_ms"].as<int>(10000);
      config.uploader.s3.request_timeout_ms =
        yaml["uploader"]["s3"]["request_timeout_ms"].as<int>(300000);
    }

    if (yaml["uploader"]["retry"]) {
      config.uploader.retry.max_retries = yaml["uploader"]["retry"]["max_retries"].as<int>(5);
      config.uploader.retry.initial_delay_ms =
        std::chrono::milliseconds(yaml["uploader"]["retry"]["initial_delay_ms"].as<int>(1000));
      config.uploader.retry.max_delay_ms =
        std::chrono::milliseconds(yaml["uploader"]["retry"]["max_delay_ms"].as<int>(300000));
      config.uploader.retry.backoff_base =
        yaml["uploader"]["retry"]["backoff_base"].as<double>(2.0);
    }

    if (yaml["uploader"]["cleanup_retry"]) {
      config.uploader.cleanup_retry.max_retries =
        yaml["uploader"]["cleanup_retry"]["max_retries"].as<int>(5);
      config.uploader.cleanup_retry.initial_delay_ms = std::chrono::milliseconds(
        yaml["uploader"]["cleanup_retry"]["initial_delay_ms"].as<int>(30000)
      );
      config.uploader.cleanup_retry.backoff_multiplier =
        yaml["uploader"]["cleanup_retry"]["backoff_multiplier"].as<double>(2.0);
      const auto cleanup_max_delay_ms =
        yaml["uploader"]["cleanup_retry"]["max_delay_ms"].as<int>(600000);
      config.uploader.cleanup_retry.max_delay_ms = std::chrono::milliseconds(cleanup_max_delay_ms);
    }
  }

  if (config.ws.url.empty()) {
    config.ws.url = "ws://localhost:8090/transfer";
  }

  return config;
}

}  // namespace transfer
}  // namespace axon

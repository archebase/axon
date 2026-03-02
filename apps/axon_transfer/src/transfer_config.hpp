// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_CONFIG_HPP
#define AXON_TRANSFER_CONFIG_HPP

#include <chrono>
#include <string>

namespace axon {
namespace transfer {

struct WsConfig {
  std::string url{"ws://localhost:8090/transfer"};
  std::chrono::milliseconds ping_interval_ms{30000};
  std::chrono::milliseconds ping_timeout_ms{10000};

  struct ReconnectConfig {
    std::chrono::milliseconds initial_delay_ms{1000};
    double backoff_multiplier{2.0};
    std::chrono::milliseconds max_delay_ms{60000};
    double jitter_factor{0.2};
  } reconnect;
};

struct ScannerConfig {
  std::string data_dir{"/tmp/axon/recording"};
  bool require_json_sidecar{true};
};

struct S3Config {
  std::string endpoint_url;
  std::string bucket;
  std::string region{"cn-northwest-1"};
  std::string access_key;
  std::string secret_key;
  uint64_t part_size{67108864};
  int connect_timeout_ms{10000};
  int request_timeout_ms{300000};
};

struct RetryConfig {
  int max_retries{5};
  std::chrono::milliseconds initial_delay_ms{1000};
  std::chrono::milliseconds max_delay_ms{300000};
  double backoff_base{2.0};
};

struct UploaderConfig {
  std::string state_db_path{"/tmp/axon/transfer/transfer_state.db"};
  bool delete_after_upload{true};
  std::string failed_uploads_dir{"/tmp/axon/transfer/failed_uploads/"};
  int num_workers{2};
  S3Config s3;
  RetryConfig retry;
};

struct TransferConfig {
  std::string device_id;
  std::string factory_id;

  WsConfig ws;
  ScannerConfig scanner;
  UploaderConfig uploader;
};

TransferConfig load_config(const std::string& yaml_path);

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_CONFIG_HPP

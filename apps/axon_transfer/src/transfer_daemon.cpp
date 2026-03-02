// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "transfer_daemon.hpp"

#include <boost/filesystem.hpp>

#include <iostream>

#include "edge_uploader.hpp"
#include "retry_handler.hpp"
#include "s3_client.hpp"
#include "upload_state_manager.hpp"

namespace axon {
namespace transfer {

namespace fs = boost::filesystem;

TransferDaemon::TransferDaemon(const TransferConfig& config)
    : config_(config)
    , signals_(ioc_, SIGINT, SIGTERM) {
  signals_.async_wait([this](const boost::system::error_code& ec, int signal) {
    if (!ec) {
      std::cout << "Received signal " << signal << ", shutting down...\n";
      stop();
    }
  });

  std::string db_dir = config_.uploader.state_db_path;
  auto last_slash = db_dir.find_last_of("/\\");
  if (last_slash != std::string::npos) {
    fs::create_directories(db_dir.substr(0, last_slash));
  }
  fs::create_directories(config_.uploader.failed_uploads_dir);
  fs::create_directories(config_.scanner.data_dir);

  state_manager_ =
    std::make_shared<axon::uploader::UploadStateManager>(config_.uploader.state_db_path);

  axon::uploader::UploaderConfig uploader_config;
  uploader_config.state_db_path = config_.uploader.state_db_path;
  uploader_config.delete_after_upload = config_.uploader.delete_after_upload;
  uploader_config.failed_uploads_dir = config_.uploader.failed_uploads_dir;
  uploader_config.num_workers = config_.uploader.num_workers;
  uploader_config.s3.endpoint_url = config_.uploader.s3.endpoint_url;
  uploader_config.s3.bucket = config_.uploader.s3.bucket;
  uploader_config.s3.region = config_.uploader.s3.region;
  uploader_config.s3.access_key = config_.uploader.s3.access_key;
  uploader_config.s3.secret_key = config_.uploader.s3.secret_key;
  uploader_config.s3.part_size = config_.uploader.s3.part_size;
  uploader_config.s3.connect_timeout_ms = config_.uploader.s3.connect_timeout_ms;
  uploader_config.s3.request_timeout_ms = config_.uploader.s3.request_timeout_ms;
  uploader_config.retry.max_retries = config_.uploader.retry.max_retries;
  uploader_config.retry.initial_delay = config_.uploader.retry.initial_delay_ms;
  uploader_config.retry.max_delay = config_.uploader.retry.max_delay_ms;
  uploader_config.retry.exponential_base = config_.uploader.retry.backoff_base;

  uploader_ = std::make_shared<axon::uploader::EdgeUploader>(uploader_config);
  uploader_->start();

  scanner_ = std::make_unique<FileScanner>(config_.scanner, state_manager_);

  WsConfig ws_config = config_.ws;
  ws_config.url += "/" + config_.device_id;
  ws_client_ = std::make_unique<WsClient>(ioc_, ws_config);

  coordinator_ = std::make_unique<UploadCoordinator>(
    config_, *ws_client_, *scanner_, uploader_, state_manager_, ioc_
  );
}

void TransferDaemon::run() {
  ws_client_->set_connect_handler([this]() {
    on_ws_connected();
  });
  ws_client_->set_disconnect_handler([this]() {
    on_ws_disconnected();
  });
  ws_client_->set_message_handler([this](const nlohmann::json& msg) {
    on_ws_message(msg);
  });

  std::string ws_url = config_.ws.url + "/" + config_.device_id;
  std::cout << "axon_transfer: connecting to " << ws_url << "...\n";

  ws_client_->start();

  ioc_.run();
}

void TransferDaemon::stop() {
  ws_client_->stop();
  uploader_->stop();
  ioc_.stop();
}

void TransferDaemon::on_ws_connected() {
  std::cout << "axon_transfer: connected to fleet server\n";
  coordinator_->send_connected();
}

void TransferDaemon::on_ws_disconnected() {
  std::cout << "axon_transfer: disconnected from fleet server\n";
}

void TransferDaemon::on_ws_message(const nlohmann::json& msg) {
  std::string type = msg.value("type", "");

  if (type == "upload_request") {
    std::string task_id = msg.value("task_id", "");
    int priority = msg.value("priority", 0);
    coordinator_->on_upload_request(task_id, priority);
  } else if (type == "upload_all") {
    coordinator_->on_upload_all();
  } else if (type == "cancel") {
    std::string task_id = msg.value("task_id", "");
    coordinator_->on_cancel(task_id);
  } else if (type == "status_query") {
    coordinator_->on_status_query();
  } else if (type == "upload_ack") {
    std::string task_id = msg.value("task_id", "");
    coordinator_->on_upload_ack(task_id);
  } else {
    std::cerr << "axon_transfer: unknown message type: " << type << "\n";
  }
}

}  // namespace transfer
}  // namespace axon

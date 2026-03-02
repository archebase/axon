// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_DAEMON_HPP
#define AXON_TRANSFER_DAEMON_HPP

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>

#include <memory>
#include <string>

#include "file_scanner.hpp"
#include "transfer_config.hpp"
#include "upload_coordinator.hpp"
#include "ws_client.hpp"

namespace axon {
namespace uploader {
class EdgeUploader;
class UploadStateManager;
}  // namespace uploader

namespace transfer {

class TransferDaemon {
public:
  explicit TransferDaemon(const TransferConfig& config);

  void run();
  void stop();

private:
  void on_ws_connected();
  void on_ws_disconnected();
  void on_ws_message(const nlohmann::json& msg);

  TransferConfig config_;

  boost::asio::io_context ioc_;
  boost::asio::signal_set signals_;

  std::shared_ptr<axon::uploader::UploadStateManager> state_manager_;
  std::shared_ptr<axon::uploader::EdgeUploader> uploader_;
  std::unique_ptr<FileScanner> scanner_;
  std::unique_ptr<WsClient> ws_client_;
  std::unique_ptr<UploadCoordinator> coordinator_;
};

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_DAEMON_HPP

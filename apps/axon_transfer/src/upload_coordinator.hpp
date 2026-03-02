// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_UPLOAD_COORDINATOR_HPP
#define AXON_TRANSFER_UPLOAD_COORDINATOR_HPP

#include <boost/asio.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "file_scanner.hpp"
#include "transfer_config.hpp"
#include "ws_client.hpp"

namespace axon {
namespace uploader {
class EdgeUploader;
class UploadStateManager;
struct UploadRecord;
}  // namespace uploader

namespace transfer {

class UploadCoordinator {
public:
  UploadCoordinator(
    const TransferConfig& config, WsClient& ws_client, FileScanner& scanner,
    std::shared_ptr<axon::uploader::EdgeUploader> uploader,
    std::shared_ptr<axon::uploader::UploadStateManager> state_manager, boost::asio::io_context& ioc
  );

  void on_upload_request(const std::string& task_id, int priority = 0);
  void on_upload_all();
  void on_cancel(const std::string& task_id);
  void on_status_query();
  void on_upload_ack(const std::string& task_id);

  void send_connected();
  void send_status();

private:
  void enqueue(const FileGroup& group, int priority);
  void on_upload_started(
    const std::string& task_id, const std::vector<std::string>& files, uint64_t total_bytes
  );
  void on_upload_complete(const std::string& task_id, bool success, const std::string& error);
  void schedule_cleanup_retry(const axon::uploader::UploadRecord& record, const std::string& error);
  void run_cleanup_retry_pass();
  void schedule_next_cleanup_timer();
  std::chrono::milliseconds cleanup_delay_for_retry(int retry_count) const;

  const TransferConfig& config_;
  WsClient& ws_client_;
  FileScanner& scanner_;
  std::shared_ptr<axon::uploader::EdgeUploader> uploader_;
  std::shared_ptr<axon::uploader::UploadStateManager> state_manager_;
  boost::asio::io_context& ioc_;
  boost::asio::steady_timer cleanup_retry_timer_;
};

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_UPLOAD_COORDINATOR_HPP

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_TRANSFER_UPLOAD_OBSERVABILITY_HPP
#define AXON_TRANSFER_UPLOAD_OBSERVABILITY_HPP

#include <nlohmann/json.hpp>

#include "edge_uploader.hpp"
#include "transfer_config.hpp"
#include "upload_state_manager.hpp"

namespace axon {
namespace transfer {

nlohmann::json build_upload_observability_status(
  const TransferConfig& config, axon::uploader::UploadStateManager& state_manager,
  const axon::uploader::UploaderStats& stats
);

}  // namespace transfer
}  // namespace axon

#endif  // AXON_TRANSFER_UPLOAD_OBSERVABILITY_HPP

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_PANEL_PANEL_API_HPP
#define AXON_PANEL_PANEL_API_HPP

#include <string>

namespace httplib {
class Server;
}

namespace axon {
namespace panel {

struct PanelApiOptions {
  std::string config_dir;
  std::string recording_dir;
};

PanelApiOptions make_default_panel_api_options();

void register_panel_api(httplib::Server& server, const PanelApiOptions& options);

}  // namespace panel
}  // namespace axon

#endif  // AXON_PANEL_PANEL_API_HPP

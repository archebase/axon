# Axon Debian Packaging

<!--
SPDX-FileCopyrightText: 2026 ArcheBase
SPDX-License-Identifier: MulanPSL-2.0
-->

This directory contains Debian packaging configuration for Axon.

## Directory Structure

```
packaging/deb/
├── common/           # Shared files (dirs, install, docs)
├── recorder/         # axon-recorder package
├── config/           # axon-config package
├── dispatcher/       # axon-dispatcher package
├── agent/            # axon-agent package
├── system/           # axon-system package
├── meta-all/         # axon-all meta-package
├── plugin-ros1/      # axon-plugin-ros1 package
├── plugin-ros2/      # axon-plugin-ros2 package
├── plugin-udp/       # axon-plugin-udp package
└── plugin-zenoh/     # axon-plugin-zenoh package
```

## Packages

| Package | Description |
|---------|-------------|
| `axon-recorder` | Core recorder with HTTP RPC API |
| `axon-config` | Robot configuration CLI tool |
| `axon-panel` | Web control panel (embedded assets) |
| `axon-transfer` | S3 transfer daemon |
| `axon-system` | Host and runtime monitor |
| `axon-dispatcher` | Unified CLI dispatcher (`axon` command) |
| `axon-agent` | Orchestration and process management agent |
| `axon-all` | Meta-package installing all core tools |
| `axon-plugin-ros2` | ROS2 (Humble/Jazzy/Rolling) plugin |
| `axon-plugin-ros1` | ROS1 (Noetic) plugin |
| `axon-plugin-udp` | UDP JSON recording plugin |
| `axon-plugin-zenoh` | Zenoh middleware plugin |

## Installation Layout

- **Binaries**: `/opt/axon/bin/` (symlinked to `/usr/bin/`)
- **Libraries**: `/opt/axon/lib/`
- **Plugins**: `/opt/axon/plugins/`
- **Config**: `/opt/axon/share/config/`
- **Runtime data**: `/axon/`
- **System config**: `/etc/axon/`

## Building Packages

See [deb-packaging-design.md](../../../docs/designs/deb-packaging-design.md) for complete build instructions.

## Versioning

All packages share the same version (e.g., `0.2.1-1`) synchronized at release time.

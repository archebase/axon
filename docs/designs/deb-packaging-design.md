# Axon Deb Packaging Design

<!--
SPDX-FileCopyrightText: 2026 ArcheBase
SPDX-License-Identifier: MulanPSL-2.0
-->

This document describes the design for packaging Axon components as Debian packages.

## Overview

Axon will be distributed as multiple Debian packages installed to `/opt/axon`. Each package is independently installable based on the user's requirements. Plugin packages follow system dependencies (ROS1, ROS2, etc.) and can only be installed when the corresponding middleware is available.

## Package Structure

### Main Application Packages

| Package | Description | Runtime Dependencies | Build Dependencies |
|---------|-------------|---------------------|-------------------|
| `axon-recorder` | Core recorder with HTTP RPC API | `${shlibs:Depends}`, `libzstd1`, `liblz4-1` | `debhelper-compat (=13)`, `cmake`, `libboost-dev`, `libyaml-cpp-dev`, `libssl-dev`, `libzstd-dev`, `liblz4-dev` |
| `axon-config` | Robot configuration CLI tool | `${shlibs:Depends}`, `libzstd1`, `liblz4-1` | `debhelper-compat (=13)`, `cmake`, `libzstd-dev`, `liblz4-dev` |
| `axon-transfer` | S3 transfer daemon | `${shlibs:Depends}`, `libaws-c-common0`, `libaws-c-event-stream0` | `debhelper-compat (=13)`, `cmake`, `libboost-dev`, `libyaml-cpp-dev`, `libssl-dev`, `libaws-c-common-dev`, `libaws-c-event-stream-dev` |
| `axon-panel` | Web control panel (embedded assets) | `${shlibs:Depends}` | `debhelper-compat (=13)`, `cmake`, `nodejs (>=18)`, `npm` |
| `axon-dispatcher` | Unified CLI dispatcher (`axon` command) | None (standalone) | `debhelper-compat (=13)`, `cmake` |

### Plugin Packages

| Package | Description | System Dependencies |
|---------|-------------|---------------------|
| `axon-plugin-ros1` | ROS1 Noetic middleware plugin | ROS1 Noetic (`/opt/ros/noetic`) |
| `axon-plugin-ros2` | ROS2 middleware plugin (Humble/Jazzy/Rolling) | ROS2 (`/opt/ros/$ROS_DISTRO`) |
| `axon-plugin-udp` | UDP JSON recording plugin | None (standalone) |
| `axon-plugin-zenoh` | Zenoh middleware plugin | `libzenohc.so` |

### Meta Package

| Package | Description | Dependencies |
|---------|-------------|--------------|
| `axon-all` | Installs all core Axon tools (no plugins) | `axon-recorder`, `axon-config`, `axon-panel`, `axon-transfer`, `axon-dispatcher` |

**Note:** Plugin packages must be installed separately based on your needs.

### Unified CLI Dispatcher

| Binary | Description |
|--------|-------------|
| `axon` | Unified CLI dispatcher (git-style) that forwards to subcommands |

The `axon` dispatcher provides a unified interface similar to `git`:

```bash
# All commands are equivalent:
axon recorder [args]      → /opt/axon/bin/axon-recorder [args]
axon config [args]        → /opt/axon/bin/axon-config [args]
axon transfer [args]      → /opt/axon/bin/axon-transfer [args]
axon panel [args]         → /opt/axon/bin/axon-panel [args]
axon version              → Show all tool versions

# Traditional commands also work:
axon-recorder [args]
axon-config [args]
```

**Benefits:**
- Single entry point for all Axon tools
- Consistent help system (`axon --help`, `axon recorder --help`)
- Easier discovery of available commands
- Tab completion support

## Installation Directory Structure

```
/opt/axon/
├── bin/                          # Executables (symlinked to /usr/bin)
│   ├── axon                      # Unified CLI dispatcher
│   ├── axon-recorder
│   ├── axon-config
│   ├── axon-transfer
│   └── axon-panel
├── lib/                          # Core libraries
│   ├── libaxon_logging.so
│   ├── libaxon_mcap.so
│   ├── libaxon_uploader.so
│   └── libaxon_transfer_core.so
├── plugins/                      # Middleware plugins
│   ├── libaxon_ros1_plugin.so
│   ├── libaxon_ros2_plugin.so
│   ├── libaxon_udp_plugin.so
│   └── libaxon_zenoh_plugin.so
├── share/                        # Shared data
│   ├── config/                   # Default configurations
│   │   └── default.yaml
│   ├── systemd/                 # systemd service files
│   │   ├── axon-recorder@.service
│   │   └── axon-transfer.service
│   └── doc/                     # Documentation
│       └── axon/
│           └── README.html

/usr/bin/                         # User-facing symlinks
├── axon -> /opt/axon/bin/axon
├── axon-recorder -> /opt/axon/bin/axon-recorder
├── axon-config -> /opt/axon/bin/axon-config
├── axon-transfer -> /opt/axon/bin/axon-transfer
└── axon-panel -> /opt/axon/bin/axon-panel

/usr/share/bash-completion/completions/  # Shell completions
├── axon                         # Dispatcher completion
├── axon-recorder                # Recorder completion
├── axon-config                  # Config tool completion
├── axon-transfer                # Transfer daemon completion
└── axon-panel                   # Panel completion

/etc/axon/                        # Runtime configuration (not packaged)
└── config.yaml                   # User configuration

/axon/                            # Runtime data (working directory)
├── cache/                        # Config cache
├── uploads/                      # Upload queue
└── recordings/                   # Default recording output

/var/lib/axon/                    # State data
├── transfer/                     # Transfer daemon state

/var/log/axon/                    # Log files
├── axon-recorder.log
└── axon-transfer.log
```

## Implementation Plan

**Status:**
- ✅ Step 1: Create Packaging Infrastructure
- ✅ Step 2: Update CMake Installation Rules
- ✅ Step 3: Create Docker Build Environment
- ✅ Step 4: Create Build Scripts
- ✅ Step 5: Package Configuration Templates
- ✅ Step 6: systemd Service Units
- ✅ Step 7: Web Panel Build Integration
- ✅ Step 8: Makefile Integration
- ✅ Step 9: Package Dependency Declarations
- ✅ Step 10: CI/CD Integration
- ✅ Step 11: Create Unified CLI Dispatcher
- ✅ Step 12: Create Meta Package (axon-all)

**All packaging steps complete!** 🎉

---

### Step 1: Create Packaging Infrastructure

**Location:** `packaging/deb/`

Create the directory structure:
```
packaging/deb/
├── common/                       # Shared files
│   ├── axon.dirs                # Directory list
│   ├── axon.install             # Install paths
│   └── axon-docs.docs           # Documentation
├── recorder/
│   └── debian/                  # axon-recorder package
│       ├── control
│       ├── postinst
│       ├── prerm
│       └── rules
├── config/
│   └── debian/                  # axon-config package
├── transfer/
│   └── debian/                  # axon-transfer package
├── panel/
│   └── debian/                  # axon-panel package
├── dispatcher/
│   └── debian/                  # axon-dispatcher package
├── meta-all/
│   └── debian/                  # axon-all meta-package
│       ├── control
│       ├── changelog
│       └── rules
├── plugin-ros1/
│   └── debian/                  # axon-plugin-ros1 package
├── plugin-ros2/
│   └── debian/                  # axon-plugin-ros2 package
├── plugin-udp/
│   └── debian/                  # axon-plugin-udp package
└── plugin-zenoh/
    └── debian/                  # axon-plugin-zenoh package
```

**Tasks:**
1. Create `packaging/deb/common/` with shared packaging scripts
2. Create debian/ directory for each package
3. Add `debian/control` with package metadata
4. Add `debian/rules` with build instructions
5. Add maintainer scripts (`postinst`, `prerm`) where needed

### Step 2: Update CMake Installation Rules

**Modified files:** Multiple `CMakeLists.txt` files

Update all CMakeLists.txt to use `CMAKE_INSTALL_PREFIX` set to `/opt/axon`:

```cmake
# In each component's CMakeLists.txt
install(TARGETS axon_recorder
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(TARGETS axon_ros2_plugin
    LIBRARY DESTINATION plugins
)
```

**Tasks:**
1. Update `apps/axon_recorder/CMakeLists.txt`
2. Update `apps/axon_config/CMakeLists.txt`
3. Update `apps/axon_transfer/CMakeLists.txt`
4. Update `apps/axon_dispatcher/CMakeLists.txt` (new)
5. Update `middlewares/ros2/CMakeLists.txt` (plugin install)
6. Update `middlewares/ros1/CMakeLists.txt` (plugin install)
7. Update `middlewares/udp/CMakeLists.txt` (plugin install)
8. Update `middlewares/zenoh/CMakeLists.txt` (plugin install)

### Step 3: Create Docker Build Environment

**Location:** `docker/Dockerfile.package-ros2.humble`, `docker/Dockerfile.package-ros2.jazzy`, etc.

Create multi-stage Dockerfiles for building packages:

```dockerfile
# Stage 1: Build environment
FROM ubuntu:22.04 AS builder
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    debhelper \
    dpkg-dev \
    # ROS dependencies
    # Other dependencies

# Stage 2: Package build
FROM builder AS package-builder
COPY . /axon
WORKDIR /axon
RUN mk-build-deps -i
RUN dpkg-buildpackage -b -uc -us

# Stage 3: Package output
FROM scratch AS output
COPY --from=package-builder /axon/*.deb /
```

**Tasks:**
1. Create `docker/Dockerfile.package-base` (base build environment)
2. Create `docker/Dockerfile.package-ros2.humble` (ROS2 Humble packages)
3. Create `docker/Dockerfile.package-ros2.jazzy` (ROS2 Jazzy packages)
4. Create `docker/Dockerfile.package-ros1` (ROS1 Noetic packages)
5. Create `docker/Dockerfile.package-standalone` (non-ROS packages)

### Step 4: Create Build Scripts

**Location:** `packaging/deb/scripts/`

Create helper scripts for building packages:

| Script | Purpose |
|--------|---------|
| `build-all.sh` | Build all packages for current environment |
| `build-ros2.sh` | Build ROS2 plugin packages |
| `build-ros1.sh` | Build ROS1 plugin packages |
| `build-standalone.sh` | Build non-ROS packages (recorder, config, panel) |
| `build-in-docker.sh` | Build packages inside Docker container |

**Tasks:**
1. Create `packaging/deb/scripts/build-standalone.sh`
2. Create `packaging/deb/scripts/build-ros2.sh`
3. Create `packaging/deb/scripts/build-ros1.sh`
4. Create `packaging/deb/scripts/build-all.sh`
5. Create `packaging/deb/scripts/build-in-docker.sh`

### Step 5: Package Configuration Templates

**Location:** `packaging/deb/config/`

Add default configuration files:

```
packaging/deb/config/
├── axon-recorder.yaml           # Default recorder config
└── axon-transfer.yaml           # Default transfer daemon config
```

**Tasks:**
1. Create default configuration templates
2. Add configuration to `debian/rules` installation
3. Document configuration options in package docs

### Step 6: systemd Service Units

**Location:** `packaging/deb/systemd/`

Create systemd service files. The recorder uses **instantiated services** to support different plugins:

```ini
# /opt/axon/share/systemd/axon-recorder@.service (template)
[Unit]
Description=Axon ROS Recorder (%i)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/opt/axon/bin/axon-recorder --plugin /opt/axon/plugins/%i
Restart=on-failure
User=axon
Group=axon
RestartSec=5

# Security hardening
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/axon /var/lib/axon /var/log/axon

[Install]
WantedBy=multi-user.target
```

**Usage:**
```bash
# Enable with specific plugin
sudo systemctl enable axon-recorder@libaxon_ros2_plugin.service
sudo systemctl start axon-recorder@libaxon_ros2_plugin.service

# Or use symbolic names via symlinks
sudo systemctl enable axon-recorder@ros2  # → libaxon_ros2_plugin.so
```

```ini
# /opt/axon/share/systemd/axon-transfer.service
[Unit]
Description=Axon S3 Transfer Daemon
After=network-online.target axon-recorder@ros2.service
Wants=network-online.target

[Service]
Type=simple
ExecStart=/opt/axon/bin/axon-transfer --config /etc/axon/transfer.yaml
Restart=on-failure
User=axon
Group=axon
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Tasks:**
1. Create `axon-recorder@.service` template
2. Create `axon-transfer.service`
3. Create plugin symlink helper script (`/opt/axon/bin/axon-recorder-enable`)
4. Add service installation to `debian/postinst`
5. Create `axon` user and group in postinst with `--system --no-create-home`

### Step 7: Web Panel Build Integration

**Location:** `apps/axon_panel/`

The web panel uses **embedded static assets** - Vue 3 files are compiled and embedded into the C++ binary as compressed data (already implemented).

**Build process:**
1. Build Vue 3 assets with `npm run build`
2. Embed assets in C++ binary via `embedded_assets.cpp` (already implemented)
3. Install `axon-panel` binary to `/opt/axon/bin/`

**No separate static files needed** - the panel is fully self-contained.

**Tasks:**
1. Update `apps/axon_panel/CMakeLists.txt` for install
2. Add npm build step to `debian/rules` for axon-panel
3. Verify embedded assets work correctly

### Step 8: Makefile Integration

**Location:** `Makefile` (project root)

Add packaging targets:

```makefile
# Package building
PACKAGE_DIR := packaging/deb

.PHONY: package-standalone
package-standalone:  # Build non-ROS packages
	$(PACKAGE_DIR)/scripts/build-standalone.sh

.PHONY: package-ros2
package-ros2:  # Build ROS2 packages
	$(PACKAGE_DIR)/scripts/build-ros2.sh

.PHONY: package-ros1
package-ros1:  # Build ROS1 packages
	$(PACKAGE_DIR)/scripts/build-ros1.sh

.PHONY: package-all
package-all:  # Build all applicable packages
	$(PACKAGE_DIR)/scripts/build-all.sh
```

**Tasks:**
1. Add package targets to root Makefile
2. Add package cleaning targets
3. Document package building in README

### Step 9: Package Dependency Declarations

**Location:** `packaging/deb/*/debian/control`

Define package dependencies correctly:

```
# axon-recorder control
Package: axon-recorder
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}, libzstd1, liblz4-1
Description: Axon ROS Recorder

# axon-plugin-ros2 control
Package: axon-plugin-ros2
Architecture: any
Depends: axon-recorder, ${shlibs:Depends}, ${misc:Depends}
Description: ROS2 plugin for Axon Recorder
```

**Tasks:**
1. Define dependencies for axon-recorder
2. Define dependencies for axon-config
3. Define dependencies for axon-transfer
4. Define dependencies for axon-panel
5. Define dependencies for each plugin package

### Step 10: CI/CD Integration

**Location:** `.github/workflows/package.yml`

Create GitHub Actions workflow for building packages:

```yaml
name: Build Debian Packages

on:
  push:
    tags: ['v*']
  workflow_dispatch:

jobs:
  package-ros2-humble:
    runs-on: ubuntu-latest
    container:
      image: ubuntu:22.04
    steps:
      - uses: actions/checkout@v4
      - name: Build ROS2 Humble packages
        run: make package-docker ROS_DISTRO=humble
      - uses: actions/upload-artifact@v4
        with:
          name: deb-ros2-humble
          path: packaging/deb/output/*.deb
```

**Tasks:**
1. Create `.github/workflows/package.yml`
2. Add matrix builds for ROS1, ROS2 Humble/Jazzy/Rolling
3. Configure artifact uploads
4. Add package signing step (optional)

### Step 11: Create Unified CLI Dispatcher

**Location:** `apps/axon_dispatcher/`

Create a unified CLI dispatcher that forwards commands to individual tools:

```cpp
// apps/axon_dispatcher/axon.cpp
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <unistd.h>

const struct Command {
    const char* name;
    const char* binary;
} COMMANDS[] = {
    {"recorder", "/opt/axon/bin/axon-recorder"},
    {"config", "/opt/axon/bin/axon-config"},
    {"transfer", "/opt/axon/bin/axon-transfer"},
    {"panel", "/opt/axon/bin/axon-panel"},
    {nullptr, nullptr}
};

void show_help() {
    std::cout << "Axon - High-performance ROS recorder\n\n";
    std::cout << "Usage: axon <command> [args]\n\n";
    std::cout << "Available commands:\n";
    for (int i = 0; COMMANDS[i].name; ++i) {
        std::cout << "  " << COMMANDS[i].name << "\n";
    }
    std::cout << "  version\n";
    std::cout << "  help\n";
    std::cout << "\nSee 'axon <command> --help' for more information.\n";
}

void show_version() {
    std::cout << "Axon version " << AXON_VERSION << "\n\n";
    std::cout << "Tool versions:\n";
    // Run each tool with --version flag
    for (int i = 0; COMMANDS[i].name; ++i) {
        std::string cmd = std::string(COMMANDS[i].binary) + " --version 2>/dev/null";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char line[128];
            if (fgets(line, sizeof(line), pipe)) {
                std::cout << "  " << line;
            }
            pclose(pipe);
        }
    }
    // Scan and display plugin versions
    std::cout << "\nPlugins:\n";
    // Scan /opt/axon/plugins/ for .so files and extract version info
}
```

**Directory structure:**
```
apps/axon_dispatcher/
├── CMakeLists.txt
├── axon.cpp                 # Main dispatcher
└── debian/                  # Package configuration
    └── control
```

**Tasks:**
1. Create `apps/axon_dispatcher/axon.cpp` dispatcher implementation
2. Create `apps/axon_dispatcher/CMakeLists.txt`
3. Add version aggregation command (`axon version`)
4. Add bash completion script in `apps/axon_dispatcher/completion.bash`
5. Install `axon` binary to `/opt/axon/bin/` with symlink to `/usr/bin/axon`

### Step 12: Create Meta Package (axon-all)

**Location:** `packaging/deb/meta-all/debian/`

Create a meta-package that installs all core Axon tools (without plugins):

```
packaging/deb/meta-all/
└── debian/
    ├── control               # Package definition
    └── changelog             # Package changelog
```

**Control file:**
```
Source: axon-all
Section: utils
Priority: optional
Maintainer: ArcheBase <noreply@archebase.com>

Package: axon-all
Architecture: any
Depends: axon-recorder, axon-config, axon-panel,
         axon-transfer, axon-dispatcher,
         ${misc:Depends}
Description: Axon - Complete installation (core tools only)
 This meta-package installs all core Axon tools:
 - axon-recorder: Core recording engine
 - axon-config: Robot configuration tool
 - axon-panel: Web-based control interface
 - axon-transfer: S3 transfer daemon
 - axon-dispatcher: Unified CLI entry point
 .
 Plugin packages must be installed separately based on your needs.
```

**Tasks:**
1. Create `packaging/deb/meta-all/debian/control`
2. Create `packaging/deb/meta-all/debian/changelog`
3. Create `packaging/deb/meta-all/debian/rules` (minimal, no build needed)
4. Add meta-package to build scripts
5. Document meta-package in README

## Unified CLI Dispatcher Design

The `axon` dispatcher implements a git-style command interface:

### Command Dispatch Logic

```
┌─────────────┐
│  axon CMD   │
└──────┬──────┘
       │
       ▼
┌─────────────────────────────────┐
│  Parse command from argv[1]      │
└──────┬──────────────────────────┘
       │
       ▼
┌─────────────────────────────────┐
│  Match to known command          │
│  - recorder → axon-recorder      │
│  - config → axon-config          │
│  - transfer → axon-transfer      │
│  - panel → axon-panel            │
│  - version → aggregate versions  │
│  - help → show help              │
└──────┬──────────────────────────┘
       │
       ▼
┌─────────────────────────────────┐
│  execvp(binary, [args])          │
│  Pass through stdin/stdout/stderr│
└─────────────────────────────────┘
```

### Bash Completion

Each package installs its own completion to the standard bash-completion directory:

**Dispatcher completion** (`/usr/share/bash-completion/completions/axon`):
```bash
# /usr/share/bash-completion/completions/axon
_axon_completion() {
    local cur prev commands
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    commands="recorder config transfer panel version help"

    if [[ ${COMP_CWORD} -eq 1 ]] ; then
        COMPREPLY=( $(compgen -W "${commands}" -- "${cur}") )
        return 0
    fi

    # Delegate to command-specific completion via dynamic loading
    local completion_file="/usr/share/bash-completion/completions/axon-${prev}"
    if [[ -f "$completion_file" ]]; then
        source "$completion_file"
        local cmd_func="_axon_${prev//-/_}_completion"
        if declare -f "$cmd_func" > /dev/null; then
            "$cmd_func"
        fi
    fi
}
complete -F _axon_completion axon
```

**Each subcommand package installs its own completion:**
- `/usr/share/bash-completion/completions/axon-recorder` (from axon-recorder package)
- `/usr/share/bash-completion/completions/axon-config` (from axon-config package)
- `/usr/share/bash-completion/completions/axon-transfer` (from axon-transfer package)
- `/usr/share/bash-completion/completions/axon-panel` (from axon-panel package)

### Version Aggregation

The `axon version` command aggregates versions from all installed tools:

```bash
$ axon version
Axon version 0.2.1

Tool versions:
  axon-recorder    0.2.1
  axon-config      0.2.1
  axon-panel       0.2.1
  axon-transfer    0.2.1

Plugins:
  libaxon_ros2_plugin.so    0.2.1
  libaxon_udp_plugin.so     0.2.1
```

**Detection mechanism:**
1. Tools: Check for binaries at `/opt/axon/bin/axon-*`, execute with `--version` flag
2. Plugins: Scan `/opt/axon/plugins/` directory, read ELF `.note.gnu.build-id` or embedded version symbol
3. Missing tools/plugins are simply omitted from output

**Version synchronization:**
- All tools share the same version - synchronized at release time
- Each plugin's version must match the core recorder version
- Version mismatch warning shown if plugin is incompatible

```bash
$ axon version
Axon version 0.2.1

Tool versions:
  axon-recorder    0.2.1
  axon-config      0.2.1

Plugins:
  libaxon_ros2_plugin.so    0.2.1
  libaxon_ros1_plugin.so    0.1.5  (WARNING: version mismatch, expected 0.2.1)
```

## Package Control File Details

### axon-dispatcher

```
Source: axon-dispatcher
Section: utils
Priority: optional
Maintainer: ArcheBase <noreply@archebase.com>
Build-Depends: debhelper-compat (=13), cmake

Package: axon-dispatcher
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}
Description: Axon unified CLI dispatcher
 Provides the 'axon' command that dispatches to subcommands
 like 'axon recorder', 'axon config', etc. (similar to git).
```

### axon-all

```
Source: axon-all
Section: utils
Priority: optional
Maintainer: ArcheBase <noreply@archebase.com>

Package: axon-all
Architecture: any
Depends: axon-recorder, axon-config, axon-panel,
         axon-transfer, axon-dispatcher,
         ${misc:Depends}
Description: Axon - Complete installation (core tools only)
 This meta-package installs all core Axon tools:
 - axon-recorder: Core recording engine with HTTP RPC API
 - axon-config: Robot configuration CLI tool
 - axon-panel: Web-based control interface
 - axon-transfer: S3 transfer daemon
 - axon-dispatcher: Unified CLI entry point
 .
 Plugin packages (axon-plugin-ros2, axon-plugin-ros1, axon-plugin-udp,
 axon-plugin-zenoh) must be installed separately based on your needs.
```

### axon-recorder

```
Source: axon-recorder
Section: utils
Priority: optional
Maintainer: ArcheBase <noreply@archebase.com>
Build-Depends: debhelper-compat (=13), cmake, libboost-dev, libyaml-cpp-dev, libssl-dev, libzstd-dev, liblz4-dev

Package: axon-recorder
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}
Description: High-performance ROS recorder with HTTP RPC API
 Axon records ROS messages to MCAP format with lock-free
 queues and plugin-based middleware support.
```

### axon-plugin-ros2

```
Package: axon-plugin-ros2
Architecture: any
Depends: axon-recorder, ros-$ROS_DISTRO-ros-base, ${shlibs:Depends}, ${misc:Depends}
Description: ROS2 plugin for Axon Recorder
 This plugin enables Axon to record from ROS2 Humble/Jazzy/Rolling.
```

### axon-panel

```
Package: axon-panel
Architecture: any
Depends: axon-recorder (>= 0.2.0), ${shlibs:Depends}, ${misc:Depends}
Description: Web control panel for Axon Recorder
 Browser-based interface for monitoring and controlling
 the Axon recorder via HTTP RPC.
```

## Versioning

All packages share the **same version** - synchronized at release time:

- `0.2.1-1` - Upstream version 0.2.1, Debian revision 1
- All tools (recorder, config, panel, transfer, dispatcher) use version 0.2.1
- All plugins use version 0.2.1
- Debian revision (`-1`, `-2`, etc.) increments for packaging-only changes

**Single version source:** Version defined in root project and propagated to all components via CMake.

## Build Commands

All package builds use Docker for consistent, reproducible builds across different environments.

```bash
# Build all packages in Docker (recommended)
make package-all

# Build core packages only in Docker
make package-core

# Build all plugin packages in Docker (ROS1 + ROS2 + UDP)
make package-plugins

# Clean package build artifacts
make package-clean
```

**Output:** Built packages are placed in `packaging/deb/output/`

**Note:** The `build-in-docker.sh` script handles multi-distro builds, creating separate packages for:
- Ubuntu 20.04 (focal) - ROS1 Noetic
- Ubuntu 22.04 (jammy) - ROS2 Humble
- Ubuntu 24.04 (noble) - ROS2 Jazzy/Rolling + standalone packages

## Testing Packages

```bash
# Install built package
sudo dpkg -i packaging/deb/output/axon-recorder_0.2.1-1_amd64.deb

# Verify installation
dpkg -L axon-recorder          # List installed files
/opt/axon/bin/axon-recorder --version

# Test systemd service
sudo systemctl start axon-recorder
sudo systemctl status axon-recorder
```

## Multi-Arch Support

Packages are built for multiple architectures using Docker:
- `amd64` (x86_64) - Primary architecture
- `arm64` (aarch64) - For ARM-based devices (Raspberry Pi, NVIDIA Jetson, etc.)

The build system uses Docker's multi-platform capabilities to build for different architectures.

## Usage Examples

### Complete Installation (axon-all)

```bash
# Install core tools with one command
sudo apt install ./axon-all_0.2.1-1_amd64.deb

# This installs:
# - axon-recorder
# - axon-config
# - axon-panel
# - axon-transfer
# - axon-dispatcher

# Install plugins separately based on your needs:
sudo apt install ./axon-plugin-ros2_0.2.1-1_amd64.deb
```

### Using the Unified CLI

```bash
# Start the recorder (requires a plugin)
axon recorder --plugin /opt/axon/plugins/libaxon_ros2_plugin.so

# Configure the robot
axon config init
axon config scan

# Start the transfer daemon
axon transfer --config /etc/axon/transfer.yaml

# Start the web panel
axon panel --port 8080

# Show versions
axon version

# Get help
axon --help
axon recorder --help
```

### Minimal Installation

```bash
# Install only the recorder (for headless setups)
sudo apt install ./axon-recorder_0.2.1-1_amd64.deb \
                 ./axon-plugin-ros2_0.2.1-1_amd64.deb

# Or install with config tool
sudo apt install ./axon-recorder_0.2.1-1_amd64.deb \
                 ./axon-config_0.2.1-1_amd64.deb \
                 ./axon-plugin-ros2_0.2.1-1_amd64.deb
```

### Traditional Commands Still Work

```bash
# All equivalent commands:
axon recorder --simple          # Using dispatcher
axon-recorder --simple          # Direct binary
/opt/axon/bin/axon-recorder --simple  # Full path
```

## Package Removal Behavior

### Removal vs Purge

Debian packages support two levels of removal:

| Command | Effect |
|---------|--------|
| `sudo apt remove axon-recorder` | Removes binaries, preserves config and data |
| `sudo apt purge axon-recorder` | Removes everything including configs |

### What Gets Preserved (remove)

The following locations are **NOT** removed on `apt remove`:
- `/etc/axon/` - User configuration files
- `/axon/` - Runtime data (recordings, cache, uploads)
- `/var/lib/axon/` - State data
- `/var/log/axon/` - Log files

### What Gets Removed

All package-managed files in `/opt/axon/`:
- `/opt/axon/bin/*` - Binaries
- `/opt/axon/lib/*` - Libraries
- `/opt/axon/plugins/*` - Plugins
- `/opt/axon/share/*` - Shared data
- `/usr/bin/axon*` - Symlinks

### systemd Service Behavior

On package removal:
- Services are stopped before removal
- Services are disabled (won't start on boot)
- Service files remain (can be manually re-enabled if package is reinstalled)

### Cleanup After Removal

To completely remove all Axon data:

```bash
# Remove package
sudo apt purge axon-all

# Remove runtime data (optional, destroys recordings!)
sudo rm -rf /axon/
sudo rm -rf /var/lib/axon/
sudo rm -rf /var/log/axon/

# Remove configuration (optional)
sudo rm -rf /etc/axon/

# Remove user (if created)
sudo userdel axon
```

### Plugin Removal Safety

When removing a plugin package, the recorder will refuse to start if a service references that plugin:

```bash
$ sudo systemctl start axon-recorder@ros2
Failed to start axon-recorder@ros2.service: Unit not found
# Plugin was removed, need to disable or reconfigure service
sudo systemctl disable axon-recorder@ros2
```

## See Also

- [CMake Architecture](cmake-architecture-design.md)
- [Middleware Plugin Architecture](middleware-plugin-architecture-design.md)
- [Frontend Design](frontend-design.md)

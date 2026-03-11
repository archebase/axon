# Axon Roadmap

This document outlines the development roadmap for the Axon project. Axon is an **in-process**, high-performance robotic data recorder with a plugin-based architecture. The core objective is to operate reliably in high-bandwidth scenarios (images/point clouds) while providing observable and maintainable recording capabilities.

## Version 0.2.0 (Current Version)

### Plugin System (Core Feature)
- ✅ **Plugin Architecture**: Middleware-agnostic core with dynamic plugin loading
  - Unified C ABI interface for easy extension
  - Runtime plugin discovery (`dlopen/dlsym`)
  - ABI version compatibility checking
  - Clear separation: core libraries have zero ROS dependencies
- ✅ **ROS1 Plugin**: Support for ROS1 Noetic (`libaxon_ros1.so`)
  - Topic subscription and message introspection
  - roscpp integration and plugin lifecycle management
- ✅ **ROS2 Plugin**: Support for ROS2 Humble/Jazzy/Rolling (`libaxon_ros2.so`)
  - rclcpp-based executor integration
  - Modern CMake targets
  - Multi-version compatibility testing

### Recording Engine
- ✅ **Task-Centric Design**: One task = one MCAP file with full lifecycle management
- ✅ **State Machine**: 4-state finite state machine (IDLE → READY → RECORDING ↔ PAUSED)
- ✅ **Queue and Concurrency Model (Current Status)**
  - Per-topic queues (SPSC/MPSC depending on implementation), high-throughput oriented
  - Goal: copy-minimized message path (minimize allocations and copies)
- ✅ **MCAP Format**
  - Zstd/LZ4 compression support
  - Metadata injection (task/device/recording info)
  - Sidecar JSON generation for quick metadata access
- ✅ **Worker Thread Pool (Current Status)**
  - Parallel topic processing for high-throughput recording

### Remote Control and Fleet Integration
- ✅ **HTTP RPC API** (RESTful)
  - `POST /rpc/config` - Set task configuration (IDLE → READY)
  - `POST /rpc/begin` - Start recording (READY → RECORDING)
  - `POST /rpc/pause` - Pause recording (RECORDING → PAUSED)
  - `POST /rpc/resume` - Resume recording (PAUSED → RECORDING)
  - `POST /rpc/end` - End recording (RECORDING/PAUSED → IDLE)
  - `POST /rpc/quit` - Shutdown recorder
  - `GET /rpc/state` - Query state and version
  - `GET /rpc/stats` - Get recording statistics
- ✅ **HTTP Callbacks**: Automatic start/finish notifications to fleet server
- ✅ **Version Management**: Unified version management from CMake (0.2.0)
  - `--version` CLI option
  - Version info in HTTP API responses

### Testing and Quality
- ✅ Comprehensive unit and integration tests
- ✅ End-to-end tests with mock plugins
- ✅ Code coverage reports (Codecov integration)
- ✅ CI/CD pipeline (GitHub Actions)
- ✅ Static analysis: cppcheck for C++, clippy for Rust
- ✅ Code formatting: Google-style clang-format
- ✅ Pre-commit hooks infrastructure

### Robot Configuration Management (axon_config)
- ✅ **CLI Configuration Tool**: Standalone tool for robot initialization and config collection
  - `init` - Initialize config directory structure
  - `scan` - Scan and cache config files into MCAP attachments
  - `enable` / `disable` - Toggle config injection for recordings
  - `status` - Show current configuration state
- ✅ **Config Injection**: Automatically embed robot config (URDF, calibration, sensor config) into MCAP recordings as attachments
- ✅ **Cache System**: Single-read MCAP cache at `/axon/config/cache.mcap` for zero per-file I/O during recording

---

## Version 0.3.0 (In Planning)

### Web Control Panel (AxonPanel)
- ✅ Frontend Interface
  - ✅ Real-time status display
  - ✅ Recording control buttons
  - ✅ Topic subscription management
  - ✅ Real-time statistics charts
- ✅ Backend Integration
  - ✅ WebSocket real-time push
  - ✅ RESTful API call encapsulation
- ✅ Deployment Support
  - ✅ Standalone deployment mode
  - ✅ Independent web server with embedded static assets

### UDP Reception Service and JSON Recording
- ✅ UDP Server
  - ✅ High-performance UDP socket reception
  - ✅ JSON data stream reception
- ✅ JSON Message Recording to MCAP
  - ✅ Generic JSON schema definition
  - ✅ JSON compression and storage


### Binary File Embedding in MCAP
- ✅ Embed Format Design
  - ✅ MCAP attachment mechanism
  - ✅ Custom schema definition
  - ✅ Metadata association
- ✅ Embed Content Types
  - ✅ Device model
  - ✅ Robot description files
  - ✅ Calibration files
  - [ ] Custom attachments

### WebSocket RPC API
- [ ] **WebSocket Server**: Bidirectional real-time communication alongside HTTP RPC
  - [ ] WebSocket endpoint at `/ws` for persistent connections
  - [ ] Support same RPC commands as HTTP API (config/begin/pause/resume/end)
  - [ ] Real-time state change notifications pushed to clients
  - [ ] Real-time statistics streaming (message counts, bandwidth, disk usage)
- [ ] **Connection Management**
  - [ ] Multiple concurrent client connections
  - [ ] Automatic reconnection handling with exponential backoff
  - [ ] Heartbeat/ping-pong for connection health monitoring
- [ ] **Message Protocol**
  - [ ] JSON-based request/response format compatible with HTTP RPC
  - [ ] Event subscription model for selective updates
  - [ ] Error handling and status codes aligned with HTTP API

### S3 Transfer Daemon (axon_transfer)
- ✅ **WebSocket Client**: Connect to fleet server for upload scheduling
  - ✅ Receive per-task and bulk upload commands
  - ✅ Report progress/completion/failure back over WebSocket
  - ✅ Device identification via URL path
- ✅ **Reconnect Strategy**: Exponential backoff with jitter (1s–60s cap, ±20%)
  - ✅ Unlimited retries — daemon never gives up
  - ✅ Uploads continue uninterrupted during disconnection
- ✅ **File Scanner**: Discover MCAP + JSON pairs in configured data directory
  - ✅ Deduplication via SQLite state (skip already-completed uploads)
  - ✅ Configurable data retention (delete or keep local copy after upload)
- ✅ **S3 Upload**: Reuse `core/axon_uploader` library
  - ✅ Multipart upload for large files (>5 MB)
  - ✅ Crash recovery via SQLite state persistence

### App Packaging
- [ ] **Native Debian Packages**: Ubuntu 20.04/22.04/24.04 `.deb` packages
  - [ ] `axon-config` package with standalone binary
  - [ ] `axon-recorder` package with bundled plugins (ROS1 for 20.04, ROS2 for 22.04/24.04)
  - [ ] `axon-transfer` package with vendored AWS SDK
  - [ ] `axon-panel` Standalone deployment
- [ ] **Portable Tarballs**: Linux x86_64 portable bundles
  - [ ] Self-contained directory layout with `run.sh` launcher
  - [ ] Vendored runtime dependencies (excluding glibc/ld-linux)
- [ ] **CMake Install Rules**: Normalize install destinations
  - [ ] Add missing install rules for core libraries (`axon_mcap`, `axon_uploader`)
  - [ ] Consistent use of `GNUInstallDirs` across all components
  - [ ] Plugin install to `/opt/axon/lib/axon/plugins/`
- [ ] **Runtime Linking**: RPATH/RUNPATH configuration
  - [ ] Set `RUNPATH=$ORIGIN/../lib/bundled:$ORIGIN/../lib` for binaries
  - [ ] Plugin-local runpath for vendored dependencies
  - [ ] Dependency vendoring script with `ldd` analysis
- [ ] **Plugin Discovery**: Default plugin resolution in recorder
  - [ ] Search `/opt/axon/lib/axon/plugins/` by default
  - [ ] Support `AXON_PLUGIN_DIR` environment variable override
  - [ ] Explicit selection via config profile or CLI flag
- [ ] **CI Packaging Matrix**: GitHub Actions workflow
  - [ ] Per-OS build containers (20.04/22.04/24.04)
  - [ ] Automated `.deb` generation with CPack
  - [ ] Release asset publishing with checksums and SBOM

---

## Version 0.4.0 (Performance Optimization)

### Memory Optimization
- [ ] Zero-Copy Optimization
  - [ ] Avoid extra copies during serialization
- [ ] Memory Pool and Preallocation
  - [ ] Message buffer pooling
  - [ ] Batch buffer reuse
  - [ ] Preallocation strategies

### Copy Optimization
- [ ] Batch Writes
  - [ ] Multi-message aggregation for single write
  - [ ] Reduce system call overhead
- [ ] Compression Optimization
  - [ ] Compression and write decoupling
  - [ ] Zstd/LZ4 compression benchmark comparison
  - [ ] Optional compression level tuning

### Blocking Backpressure
- [ ] Blocking Boundary Control
  - [ ] Allow blocking subscription threads
  - [ ] **Forbid blocking RPC/control threads**
- [ ] Thread Group Isolation
  - [ ] Recording subscriptions use separate executor/threads
  - [ ] Control plane separate thread pool

### Hybrid Recording
- [ ] Unified recording pipeline for ROS binary + UDP JSON in a single MCAP file
- [ ] Time-synchronized message interleaving across sources
- [ ] Per-source topic namespace isolation

### Robot Configuration Management via Web UI
- [ ] **AxonPanel Config Integration**
  - [ ] View current robot configuration (URDF, calibration, sensors)
  - [ ] Upload and update configuration files
  - [ ] Trigger config scan and cache rebuild
  - [ ] Enable/disable config injection toggle
  - [ ] Config change history and diff view

### Robot Registration (axon_config)
- [ ] **Keystone Integration**: Register robot identity from `axon_config` to Keystone
  - [ ] Add `register` command for Keystone enrollment flow
  - [ ] Push robot metadata (serial number/model/site) with token-based authentication
  - [ ] Persist registration status locally for retry and diagnostics

### Usability Enhancements
- [ ] Configuration Management Enhancements
  - [ ] Configuration validation and error messages
  - [ ] More configuration templates and presets
- [ ] Recording Task Management
  - [ ] Task list viewing
  - [ ] Batch operations
- [ ] Logging and Troubleshooting
  - [ ] Dynamic log level adjustment
  - [ ] Key event highlighting
  - [ ] Error diagnostic suggestions

---

## Version 0.5.0 (Future)

### Security Enhancements (Security by Default)
- [ ] HTTPS/TLS support
- [ ] API key authentication
  - [ ] Key loading and rotation workflow
- [ ] Security defaults
  - [ ] Default listen on `localhost` only
  - [ ] Callback URL allowlist

### Operations and Alerting
- [ ] Disk space threshold monitoring and alerting
- [ ] Long blocking alerting

### System Monitor Component (axon_system)
- [ ] **Standalone Monitor Service**: Add `axon_system` for host/runtime observability
  - [ ] Collect CPU, memory, disk, and network metrics
  - [ ] Expose health and resource status for recorder/transfer processes
  - [ ] Integrate alerts with existing operations pipeline

### Custom Upload Rules
- [ ] Upload Strategy Configuration
  - [ ] Topic-based filtering rules
  - [ ] Message type-based filtering rules
  - [ ] Conditional upload triggers (file size, time interval)
- [ ] Upload Priority Management
  - [ ] Critical data prioritization
  - [ ] Bandwidth throttling strategies

### Selective Topic Recording
- [ ] Regex pattern matching for topic names
- [ ] Topic blacklist/whitelist mechanisms
- [ ] On-demand subscription

---

## Version 0.6.0 (Long-term)

### Enterprise Features
- [ ] Multi-tenant support and authentication/authorization
  - [ ] JWT authentication
  - [ ] API key management

### Advanced Monitoring
- [ ] Message Integrity Monitoring
  - [ ] Sequence number tracking
  - [ ] Message gap detection and reporting
  - [ ] Packet loss statistics
  - [ ] Real-time alert notifications
- [ ] Latency Monitoring
  - [ ] End-to-end latency tracking
  - [ ] Per-topic latency percentile reporting
  - [ ] Latency anomaly detection and alerting
  - [ ] Latency hotspot analysis
- [ ] Quality of Service Monitoring
  - [ ] Bandwidth utilization
  - [ ] Queue depth monitoring

### Ecosystem and Extensibility
- [ ] Plugin SDK and Documentation
  - [ ] C/C++ SDK
  - [ ] Plugin development guide and example plugins
- [ ] Custom Integration SDK
  - [ ] Python bindings
- [ ] Recording Management CLI Tools
  - [ ] Recording inspection, batch operations, data export

---

## Version 1.0.0 (Stable Release)

### Production Ready
- [ ] Test coverage goals
  - [ ] High coverage on core recording/write/teardown/crash recovery paths
  - [ ] Coverage of critical error paths and edge cases
- [ ] Performance benchmarking and optimization
  - [ ] Baseline performance metrics
  - [ ] Comparison and migration guide with rosbag/rosbag2
- [ ] Security audit and penetration testing
  - [ ] Vulnerability assessment and hardening
- [ ] Production deployment guide
  - [ ] Best practices documentation
  - [ ] Troubleshooting guide and FAQ
- [ ] Comprehensive documentation and tutorials
  - [ ] Getting started and advanced usage guides
  - [ ] Example configurations and common scenarios

### Long-term Support
- [ ] Backward compatibility guarantee
  - [ ] Plugin ABI stability strategy
  - [ ] Deprecation policy and migration guide
- [ ] Stable plugin ABI
  - [ ] Versioning scheme
  - [ ] Compatibility testing and plugin validation tools

---

## Contribution Priorities

We welcome community contributions! Priority areas:

1. **Documentation**: Tutorials, examples, and use cases
2. **Testing**: Integration tests, edge case coverage
3. **Performance**: Benchmarking, profiling, optimization
4. **Features**: Check for issues with the `good first issue` label
5. **Bug Fixes**: Always appreciated!

See [CONTRIBUTING.md](CONTRIBUTING.md) for contribution guidelines.

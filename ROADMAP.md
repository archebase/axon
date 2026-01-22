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

---

## Version 0.3.0 (In Planning)

### Web Control Panel (AxonPanel)
- [ ] Frontend Interface
  - [ ] Real-time status display
  - [ ] Recording control buttons
  - [ ] Topic subscription management
  - [ ] Real-time statistics charts
- [ ] Backend Integration
  - [ ] WebSocket real-time push
  - [ ] RESTful API call encapsulation
- [ ] Deployment Support
  - [ ] Static resources embedded in binary
  - [ ] Standalone deployment mode

### UDP Reception Service and JSON Recording
- [ ] UDP Server
  - [ ] High-performance UDP socket reception
  - [ ] JSON data stream reception
- [ ] JSON Message Recording to MCAP
  - [ ] Generic JSON schema definition
  - [ ] JSON compression and storage
- [ ] Hybrid Recording (ROS binary + UDP JSON)

### Binary File Embedding in MCAP
- [ ] Embed Format Design
  - [ ] MCAP attachment mechanism
  - [ ] Custom schema definition
  - [ ] Metadata association
- [ ] Embed Content Types
  - [ ] Device model
  - [ ] Robot description files
  - [ ] Calibration files
  - [ ] Custom attachments

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

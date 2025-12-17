# Open Source Readiness Assessment

## Executive Summary

This document assesses the current state of the Axon project and provides recommendations for preparing it for open-source release. The goal is to make it easy for other robotics companies to adopt the SDK for their own software.

## Project Overview

**Axon** is a high-performance ROS data recorder that:
1. Records data directly in Lance format
2. Uploads Lance datasets to remote storage (edge cache or cloud)
3. Configures control flow via Dagster integration

**Target Users**: Robotics companies who want to:
- Record ROS data efficiently to Lance format
- Orchestrate data collection workflows
- Manage data pipelines with control and data flows

## Current State Analysis

### ✅ Strengths

1. **Well-Structured Core**
   - Clean separation: Rust (Lance writer) + C++ (ROS interface) + FFI bridge
   - Multi-ROS support (ROS1 Noetic, ROS2 Humble/Jazzy/Rolling)
   - Comprehensive test suite
   - Docker support for testing

2. **Good Documentation**
   - Architecture documentation
   - Design document for Dagster integration
   - README with usage examples
   - Code review guidelines

3. **Modern Build System**
   - Cargo for Rust
   - CMake for C++
   - ROS package structure
   - Makefile for convenience

### ⚠️ Gaps for Open Source

1. **Missing Open Source Essentials**
   - No LICENSE file (mentioned Apache-2.0 in package.xml but no file)
   - No CONTRIBUTING.md
   - No CODE_OF_CONDUCT.md
   - No CHANGELOG.md
   - No SECURITY.md

2. **Incomplete Project Structure**
   - Dagster integration not implemented (only design doc exists)
   - No Python package structure for Dagster
   - Missing examples/ directory
   - No schemas/ directory (like foxglove-sdk)

3. **Documentation Gaps**
   - No API documentation
   - No tutorials or getting started guides
   - No migration guide from other recorders
   - No architecture diagrams in README

4. **Package Management**
   - No PyPI package for Dagster integration
   - No crates.io publication for Rust bridge
   - No clear versioning strategy

5. **CI/CD**
   - Basic GitHub Actions workflow exists
   - Missing release automation
   - No automated publishing

## Comparison with foxglove-sdk

### foxglove-sdk Structure
```
foxglove-sdk/
├── README.md
├── CONTRIBUTING.md
├── LICENSE
├── python/          # Python SDK
├── cpp/             # C++ SDK
├── rust/            # Rust SDK
├── c/               # C SDK
├── ros/             # ROS packages
├── typescript/      # TypeScript schemas
├── schemas/         # Raw schema definitions
└── playground/      # Examples
```

### Axon Current Structure
```
axon/
├── README.md
├── ARCHITECTURE.md
├── docs/
│   └── DAGSTER_INTEGRATION_DESIGN.md
├── src/
│   ├── bridge/      # Rust FFI
│   └── cpp/         # C++ core
├── config/
├── launch/
├── docker/
└── test/
```

### Key Differences

1. **Multi-Language Support**: foxglove-sdk has separate directories for each language
2. **Schemas**: foxglove-sdk has a dedicated schemas/ directory
3. **Examples**: foxglove-sdk has a playground/ directory
4. **Packaging**: Each language has its own package structure

## Recommendations

### Phase 1: Essential Open Source Files (High Priority)

1. **LICENSE**
   - Add Apache-2.0 LICENSE file
   - Update all file headers if needed

2. **CONTRIBUTING.md**
   - Contribution guidelines
   - Development setup
   - Code style guide
   - PR process

3. **CHANGELOG.md**
   - Version history
   - Breaking changes
   - Migration guides

4. **SECURITY.md**
   - Security policy
   - Reporting process
   - Supported versions

5. **CODE_OF_CONDUCT.md**
   - Community standards
   - Enforcement process

### Phase 2: Project Structure Improvements

1. **Reorganize for Multi-Language Support**
   ```
   axon/
   ├── README.md
   ├── LICENSE
   ├── CONTRIBUTING.md
   ├── CHANGELOG.md
   ├── SECURITY.md
   ├── CODE_OF_CONDUCT.md
   │
   ├── rust/                    # Rust SDK (rename from src/bridge)
   │   ├── Cargo.toml
   │   └── src/
   │
   ├── cpp/                     # C++ SDK (rename from src/cpp)
   │   ├── CMakeLists.txt
   │   └── src/
   │
   ├── python/                  # Python SDK (NEW - for Dagster)
   │   ├── pyproject.toml
   │   ├── axon/               # Core Python package
   │   └── axon_dagster/       # Dagster integration
   │
   ├── ros/                     # ROS packages (NEW)
   │   ├── axon_msgs/          # ROS messages
   │   └── axon_bridge/        # ROS bridge (if needed)
   │
   ├── schemas/                 # Schema definitions (NEW)
   │   ├── ros/
   │   ├── protobuf/
   │   └── json/
   │
   ├── examples/                # Examples (NEW)
   │   ├── basic_recording/
   │   ├── dagster_integration/
   │   └── cloud_upload/
   │
   ├── config/                  # Configuration templates
   ├── launch/                  # ROS launch files
   ├── docker/                  # Docker images
   └── docs/                    # Documentation
   ```

2. **Create Examples Directory**
   - Basic recording example
   - Dagster integration example
   - Cloud upload example
   - Multi-topic recording example

3. **Create Schemas Directory**
   - ROS message schemas
   - Lance dataset schemas
   - Configuration schemas

### Phase 3: Documentation Enhancements

1. **API Documentation**
   - Rust API docs (cargo doc)
   - C++ API docs (Doxygen)
   - Python API docs (Sphinx)

2. **Tutorials**
   - Getting Started guide
   - Recording your first dataset
   - Setting up Dagster integration
   - Cloud upload setup

3. **Architecture Diagrams**
   - System architecture
   - Data flow diagram
   - Control flow diagram
   - Deployment diagram

### Phase 4: Package Publishing

1. **Rust Package (crates.io)**
   - Publish `axon-lance-bridge` crate
   - Version: 0.1.0 → 1.0.0

2. **Python Package (PyPI)**
   - `axon` - Core Python SDK
   - `axon-dagster` - Dagster integration
   - Version: 0.1.0

3. **ROS Packages (ROS Index)**
   - `axon` - Main ROS package
   - `axon_msgs` - Message definitions

### Phase 5: Dagster Integration Implementation

Based on the design document, implement:

1. **Python Package Structure**
   ```
   python/
   ├── pyproject.toml
   ├── axon/                    # Core SDK
   │   ├── __init__.py
   │   ├── client.py            # ROS service client
   │   └── schemas.py           # Data schemas
   │
   └── axon_dagster/            # Dagster integration
       ├── __init__.py
       ├── assets.py
       ├── ops/
       │   ├── control_flow.py
       │   ├── data_flow.py
       │   └── storage.py
       ├── resources.py
       ├── jobs.py
       ├── schedules.py
       └── sensors.py
   ```

2. **Key Components**
   - ROS service client
   - Dagster ops for recording operations
   - Asset definitions (Order, Batch, Task, Dataset)
   - Control flow jobs
   - Data flow jobs
   - Storage clients (edge cache, cloud)

### Phase 6: CI/CD Enhancements

1. **Automated Testing**
   - Multi-ROS version testing
   - Cross-platform testing
   - Integration tests

2. **Automated Publishing**
   - Release workflow
   - Package publishing
   - Documentation generation

3. **Quality Checks**
   - Code formatting
   - Linting
   - Security scanning

## Implementation Priority

### Must Have (Before Open Source)
- [ ] LICENSE file
- [ ] CONTRIBUTING.md
- [ ] CHANGELOG.md
- [ ] SECURITY.md
- [ ] Improved README with architecture diagrams
- [ ] Basic examples

### Should Have (Within 3 Months)
- [ ] Dagster integration implementation
- [ ] Python package structure
- [ ] API documentation
- [ ] Tutorials
- [ ] Package publishing setup

### Nice to Have (Future)
- [ ] Multi-language SDK structure
- [ ] Schemas directory
- [ ] Playground/examples
- [ ] Web documentation site
- [ ] Community forum/Discord

## Versioning Strategy

Recommendation: **Semantic Versioning (SemVer)**

- **Major (1.0.0)**: Breaking API changes
- **Minor (0.1.0)**: New features, backward compatible
- **Patch (0.0.1)**: Bug fixes, backward compatible

**Initial Release**: v0.1.0 (beta)
**Stable Release**: v1.0.0 (after Dagster integration)

## Naming Considerations

Current name: **Axon**

Considerations:
- ✅ Short, memorable
- ✅ Available on GitHub
- ⚠️ May conflict with other "Axon" projects
- ⚠️ Not descriptive of functionality

Alternatives (if needed):
- `axon-recorder`
- `axon-ros`
- `lance-ros-recorder`

## Next Steps

1. **Review this document** with the team
2. **Prioritize** which phases to implement first
3. **Create issues** for each phase
4. **Assign owners** for each component
5. **Set timeline** for open-source release

## Questions to Resolve

1. **Organization Name**: What GitHub org will host this? (e.g., `archebase/axon`)
2. **License**: Confirm Apache-2.0 is the right choice
3. **Naming**: Keep "Axon" or rename?
4. **Scope**: Include Dagster integration in v0.1.0 or separate package?
5. **Maintenance**: Who will maintain after open-source?
6. **Support**: How will community questions be handled?

## References

- [foxglove-sdk](https://github.com/foxglove/foxglove-sdk) - Reference structure
- [Dagster Integration Design](./DAGSTER_INTEGRATION_DESIGN.md) - Integration design
- [Architecture Document](../ARCHITECTURE.md) - System architecture

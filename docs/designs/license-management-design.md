# SPDX-FileCopyrightText: 2026 ArcheBase
#
# SPDX-License-Identifier: MulanPSL-2.0

# License Management Design

## Overview

Axon project uses a **hybrid approach** for license management, combining explicit SPDX headers in source files with REUSE.toml configuration for comprehensive coverage. This document explains the design philosophy and implementation strategy.

## Design Philosophy

### Core Principles

1. **Explicit is Better than Implicit** - Source code files MUST have SPDX headers directly in the file
2. **Configuration as Fallback** - REUSE.toml provides default licensing for non-source files
3. **Build Artifacts Ignored** - Generated files are explicitly excluded from licensing requirements
4. **Machine-Readable First** - All licensing information uses standard SPDX format

## License Declaration Strategy

### 1. C/C++ Source Files (MUST have explicit SPDX headers)

All C/C++ source files (`.cpp`, `.hpp`, `.h`) **MUST** contain SPDX headers at the top of the file:

```cpp
// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <header>

// Code continues...
```

**Rationale:**
- Source files are the core intellectual property of the project
- Explicit headers make licensing information immediately visible
- Independent of configuration files
- Survives file copying/moving to other projects

**Enforcement:**
- Pre-commit hook checks via `reuse lint`
- CI/CD pipeline validates on every push
- Files without SPDX headers will fail the build

### 2. Configuration Files (REUSE.toml provides default licensing)

Non-source files use REUSE.toml configuration for default licensing:

```toml
[[annotations]]
path = "**/*.yml"
precedence = "override"
SPDX-FileCopyrightText = "Copyright (c) 2026 ArcheBase"
SPDX-License-Identifier = "MulanPSL-2.0"
```

**Covered file types:**
- YAML files (`.yml`, `.yaml`) - CI/CD workflows, configs
- CMake files (`**/CMakeLists.txt`, `*.cmake`)
- Build scripts (`Makefile`, `**/*.sh`)
- Documentation (`**/*.md`)
- Package manifests (`**/*.xml`)

**Rationale:**
- Configuration files are boilerplate with minimal creativity
- Centralized management reduces file clutter
- Easy to update license information globally

### 3. Generated Files (Excluded from licensing)

Build artifacts and generated files are explicitly excluded:

```toml
[[annotations]]
path = "build/**"
SPDX-FileCopyrightText = "NO-COPYRIGHT"
SPDX-License-Identifier = "NO-COPYRIGHT"

[[annotations]]
path = "install/**"
SPDX-FileCopyrightText = "NO-COPYRIGHT"
SPDX-License-Identifier = "NO-COPYRIGHT"
```

**Covered patterns:**
- Build directories (`build/**`, `_deps/**`)
- Object files (`*.o`, `*.a`, `*.so`, `*.dylib`, `*.dll`)
- Installed files (`install/**`)
- Generated test mocks (`**/*_mock.*`)

## REUSE.toml Configuration Structure

The REUSE.toml file is organized into three sections:

### Section 1: Project License

```toml
[[annotations]]
path = "LICENSE"
precedence = "override"
SPDX-FileCopyrightText = "2020 Mulan PSL v2 Author"
SPDX-License-Identifier = "MulanPSL-2.0"
```

### Section 2: Source File Patterns (for explicit SPDX files)

Even though C/C++ files have explicit SPDX headers, we define patterns for consistency:

```toml
[[annotations]]
path = "**/*.cpp"
precedence = "override"
SPDX-FileCopyrightText = "Copyright (c) 2026 ArcheBase"
SPDX-License-Identifier = "MulanPSL-2.0"
```

**Note:** These serve as fallback for files that accidentally lose their headers.

### Section 3: Build Artifact Exclusions

```toml
[[annotations]]
path = "build/**"
SPDX-FileCopyrightText = "NO-COPYRIGHT"
SPDX-License-Identifier = "NO-COPYRIGHT"
```

## Workflow Integration

### 1. Creating New C/C++ Files

**Automatic (Recommended):**
```bash
# Add SPDX header when creating file
cat > new_file.cpp << EOF
// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

// Your code here
EOF
```

**Using REUSE Tool:**
```bash
# Create file normally
echo "// Your code" > new_file.cpp

# Add SPDX header
reuse annotate --year 2026 --license "MulanPSL-2.0" --copyright "ArcheBase" --style cpp new_file.cpp
```

### 2. Pre-commit Hook

The pre-commit hook ([githooks/pre-commit](../../githooks/pre-commit)) enforces compliance:

```bash
#!/bin/bash
# Check license compliance
if ! reuse lint 2>&1; then
    echo "  ✗ REUSE compliance check failed."
    echo "  Some files are missing licensing information."
    exit 1
fi
```

### 3. CI/CD Pipeline

GitHub Actions workflow ([.github/workflows/reuse.yml](../../.github/workflows/reuse.yml)):

```yaml
- name: Run REUSE check
  run: |
    reuse lint
    echo "REUSE compliance check passed!"
```

## Compliance Verification

### Check Current Status

```bash
# Quick check
reuse lint

# Verbose output
reuse lint --verbose

# Check specific directory
reuse lint core/
```

### Expected Output

```
# SUMMARY

* Bad licenses: 0
* Deprecated licenses: 0
* Licenses without file extension: 0
* Missing licenses: 0
* Unused licenses: 0
* Used licenses: MulanPSL-2.0
* Read errors: 0
* Invalid SPDX License Expressions: 0
* Files with copyright information: 183 / 183
* Files with license information: 183 / 183

Congratulations! Your project is compliant with version 3.3 of the REUSE Specification :-)
```

## Advantages of This Approach

### 1. **Clarity**
- License information is immediately visible in source files
- No need to reference external configuration
- Clear for contributors and users

### 2. **Portability**
- Source files can be copied to other projects
- SPDX headers survive repository moves
- Independent of REUSE tool availability

### 3. **Maintainability**
- Configuration files (YAML, CMake) don't need manual headers
- Easy to update licensing rules globally
- Reduced boilerplate in non-source files

### 4. **Compliance**
- Pre-commit hooks catch missing headers early
- CI/CD prevents non-compliant code from merging
- Automatic SPDX SBOM generation for releases

## Migration History

### Phase 1: Initial Setup (Completed)
- Set up REUSE.toml configuration
- Added SPDX headers to all C/C++ source files (96 files)
- Integrated with CI/CD pipeline

### Phase 2: Documentation (Completed)
- Created REUSE tool setup guide
- Documented license management strategy
- Added pre-commit hooks

### Phase 3: Full Coverage (Completed)
- Added SPDX headers to configuration files
- Ensured 100% REUSE compliance (183/183 files)
- Optimized REUSE.toml for hybrid approach

## File Coverage Statistics

| File Type | Count | License Method |
|-----------|-------|----------------|
| C/C++ source files (`.cpp`, `.hpp`, `.h`) | 96 | Explicit SPDX headers ✅ |
| Markdown documentation (`.md`) | 20+ | REUSE.toml configuration |
| YAML configs (`.yml`, `.yaml`) | 15+ | REUSE.toml configuration |
| CMake files (`CMakeLists.txt`, `*.cmake`) | 10+ | REUSE.toml configuration |
| Shell scripts (`.sh`) | 10+ | REUSE.toml configuration |
| Build artifacts (`.o`, `.so`, etc.) | N/A | Excluded (NO-COPYRIGHT) |
| **Total** | **183** | **100% REUSE Compliant** ✅ |

## Best Practices for Contributors

### 1. New C/C++ Files
Always add SPDX headers when creating new source files:
```cpp
// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0
```

### 2. Copying Code from Other Projects
- Remove original license headers
- Add Axon SPDX headers
- Verify compliance with MulanPSL-2.0

### 3. Third-Party Dependencies
- Check license compatibility
- Add to LICENSES/ directory if needed
- Update REUSE.toml if custom license

### 4. Templates and Boilerplate
- Use IDE snippets/templates with SPDX headers
- Configure editor to auto-insert license headers
- See [CONTRIBUTING.md](../../CONTRIBUTING.md) for setup instructions

## Troubleshooting

### Issue: "Missing licensing information" error

**Cause:** New C/C++ file created without SPDX header

**Solution:**
```bash
reuse annotate --year 2026 --license "MulanPSL-2.0" --copyright "ArcheBase" --style cpp path/to/file.cpp
```

### Issue: Pre-commit hook fails

**Cause:** File missing SPDX header or incorrect format

**Solution:**
```bash
# Check what's missing
reuse lint

# Add missing headers
reuse annotate --year 2026 --license "MulanPSL-2.0" --copyright "ArcheBase" <file>
```

### Issue: CI fails on REUSE check

**Cause:** License information missing or format error

**Solution:**
1. Check CI logs for specific files
2. Run `reuse lint` locally
3. Fix identified issues
4. Commit and push again

## References

- [REUSE Specification](https://reuse.software/spec/)
- [SPDX License List](https://spdx.org/licenses/)
- [Mulan PSL v2 License](../../LICENSE)
- [REUSE Tool Documentation](https://reuse.readthedocs.io/)
- [Axon Contributing Guide](../../CONTRIBUTING.md)
- [REUSE Setup Guide](../REUSE_SETUP.md)

## Summary

Axon uses a **pragmatic hybrid approach** for license management:

✅ **C/C++ source files** - Explicit SPDX headers (mandatory)
✅ **Configuration files** - REUSE.toml defaults (convenient)
✅ **Build artifacts** - Explicitly excluded (clean)
✅ **100% REUSE compliant** - Verified by automated checks

This approach balances **clarity**, **maintainability**, and **automation** to ensure proper licensing throughout the project.

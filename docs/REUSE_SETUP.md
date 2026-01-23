# REUSE Tool Setup Guide

This guide explains how to set up and use the REUSE tool for managing Mulan PSL v2 licensing in the Axon project.

## What is REUSE?

[REUSE](https://reuse.software/) is a tool provided by the Free Software Foundation Europe (FSFE) to help you comply with software licensing requirements. It:

- **Validates** license headers in source files
- **Generates** SPDX SBOM (Software Bill of Materials)
- **Automates** license header management
- **Ensures** compliance with REUSE recommendations

## Installation

### Quick Install

```bash
# Install via pip
pip install fsfe-reuse

# Verify installation
reuse --version
```

### System-wide Install (Recommended)

```bash
# Install system-wide
pip3 install fsfe-reuse --user

# Add to PATH if needed
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

# Verify
reuse --version
```

### Install via Package Manager

**Ubuntu/Debian:**
```bash
sudo apt install reuse
```

**macOS:**
```bash
brew install reuse
```

**Fedora:**
```bash
sudo dnf install reuse
```

## Configuration Files

Axon uses `reuse.toml` for REUSE configuration. The deprecated `.reuse/dep5` file has been removed.

### `reuse.toml`

TOML configuration file that provides:
- Default copyright statements for different paths
- License annotations
- Ignore patterns for build artifacts and generated files

**Important**: REUSE.toml patterns apply to files that don't have SPDX headers. For C++ and other source files, you should add SPDX headers directly to the files using the `reuse addheader` command.

## Usage

### Check License Compliance

```bash
# Check all files for licensing compliance
reuse lint

# Verbose output
reuse lint --verbose

# Specific directory
reuse lint core/
```

### Fix Licensing Issues

```bash
# Automatically fix common issues
reuse fix

# Add license header to a file
reuse addheader --year 2026 --license "MulanPSL-2.0" path/to/file.cpp

# Annotate multiple files
reuse annotate --year 2026 --license "MulanPSL2.0" path/to/directory/
```

### Generate SPDX SBOM

```bash
# Generate SPDX SBOM
reuse spdx --output-file reuse.spdx

# Generate with custom comment
reuse spdx --output-file sbom.spdx --creator "Axon Build System"
```

### Download Licenses

```bash
# Download all licenses into .reuse/downloads/
reuse download --all
```

## CI/CD Integration

### GitHub Actions

Axon includes three CI workflows for REUSE:

1. **`.github/workflows/reuse.yml`** - Standalone REUSE compliance check
   - Runs on every push and PR
   - Validates license headers
   - Generates SPDX SBOM

2. **`.github/workflows/ci.yml`** - Main CI pipeline
   - Stage 0: Format, Lint & License compliance check
   - Must pass before tests run

3. **Pre-commit Hook** - Local development
   - Runs `reuse lint` before commit
   - Prevents non-compliant commits

### Pre-commit Hook

The pre-commit hook automatically checks:
1. **Code formatting** (clang-format)
2. **License compliance** (reuse lint)

Enable it:
```bash
git config core.hooksPath githooks
```

Run manually:
```bash
./githooks/pre-commit
```

## Mulan PSL v2 in REUSE

Axon uses the **Mulan Permissive Software License v2** (MulanPSL2). REUSE identifies it via:

- **SPDX License Identifier**: `MulanPSL-2.0`
- **License File**: [LICENSE](LICENSE)
- **License Headers**: In all source files

### License Header Format

```cpp
// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0
```

**Note**: Axon uses the standard SPDX format for license headers, which is concise and machine-readable. The full text of the Mulan PSL v2 license can be found in the [LICENSE](../LICENSE) file.

## Common Issues

### Issue: "Missing licensing information"

**Solution:**
```bash
# See what's missing
reuse lint

# Fix automatically
reuse fix

# Or add header manually
reuse addheader --year 2026 --license "MulanPSL-2.0" <file>
```

### Issue: "Unused .reuse/dep5"

**Solution:**
The `.reuse/dep5` file has been deprecated and removed from the Axon project. All configuration is now in `reuse.toml`.

### Issue: "Generated files flagged"

**Solution:**
These are ignored in `reuse.toml`:
```toml
[[annotations]]
path = "build/**"
SPDX-FileCopyrightText = "NO-COPYRIGHT"
SPDX-License-Identifier = "NO-COPYRIGHT"
```

## Best Practices

1. **Always run `reuse lint` before committing**
   - Pre-commit hook enforces this
   - CI will fail if license info is missing

2. **Use `reuse addheader` for new source files**
   - Adds SPDX headers to C++, Python, and other source files
   - Example: `reuse addheader --year 2026 --license "MulanPSL-2.0" path/to/file.cpp`

3. **Keep `reuse.toml` up to date**
   - Add new directory patterns when creating new modules
   - Use directory-level patterns (`core/**`, `apps/**`) for consistency

4. **Generate SBOM for releases**
   - Run `reuse spdx` before tagging releases
   - Upload SBOM to release artifacts

## Current Status

**✅ REUSE Compliant**: The Axon project is fully compliant with REUSE Specification v3.3. All source files use the standard SPDX license header format.

To check current compliance status:
```bash
reuse lint
```

To add SPDX headers to new files:
```bash
# Add to a single file
reuse annotate --year 2026 --license "MulanPSL-2.0" --copyright "ArcheBase" path/to/file.cpp

# Add to multiple files (review changes before committing)
find . -name "*.cpp" -o -name "*.hpp" | xargs -I {} reuse annotate --year 2026 --license "MulanPSL-2.0" --copyright "ArcheBase" {}
```

## Integration with Development Workflow

### Recommended Workflow

```bash
# 1. Make your changes
git checkout -b feature-branch

# 2. Check license compliance (optional, pre-commit will do it)
reuse lint

# 3. Fix any issues
reuse fix

# 4. Stage changes
git add .

# 5. Pre-commit hook runs automatically
git commit -m "feat: add new feature"
```

### GitHub Flow

1. Push to branch
2. GitHub Actions runs `reuse.yml`
3. If REUSE check passes, PR can be merged
4. SPDX SBOM automatically generated for main branch

## Troubleshooting

### REUSE Not Found

```bash
# Install REUSE
pip install fsfe-reuse

# Or check PATH
which reuse
echo $PATH
```

### License Header Not Detected

```bash
# Check if file is ignored
reuse lint --verbose

# Verify header format
head -15 path/to/file.cpp
```

### CI Failure

Check the CI logs:
1. Go to Actions tab in GitHub
2. Click on failed workflow run
3. Expand "Run REUSE compliance check" step
4. Review error messages

## Resources

- [REUSE Official Documentation](https://reuse.readthedocs.io/)
- [Mulan PSL v2 License Text](LICENSE)
- [REUSE Specification](https://reuse.software/spec/)
- [Axon Contributing Guide](CONTRIBUTING.md)

## Quick Reference

```bash
# Check compliance
reuse lint

# Auto-fix
reuse fix

# Add header
reuse addheader --license "MulanPSL2.0" <file>

# Generate SBOM
reuse spdx --output-file sbom.spdx

# Download licenses
reuse download --all

# See all commands
reuse --help
```

## Support

For issues or questions:
1. Check [REUSE documentation](https://reuse.readthedocs.io/)
2. Review [CONTRIBUTING.md](CONTRIBUTING.md)
3. Open an issue with the label `question`

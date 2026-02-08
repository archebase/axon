# AxonConfig Design Document

**Date:** 2025-02-08
**Status:** Design

## Overview

AxonConfig is a CLI tool for managing robot configuration files that are embedded into MCAP recordings as attachments. It provides a simple interface for collecting, caching, and injecting configuration data during recording.

## Design Goals

1. **Non-Intrusive**: Does not affect recording performance when disabled
2. **Simple CLI**: Intuitive command structure for common operations
3. **Single-File Cache**: All config content embedded in one cache file - zero additional I/O during recording
4. **Directory Preservation**: Maintains original directory structure in attachments

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        AxonConfig CLI                                │
├──────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Commands:                                                              │
│   ├── init      - Create /axon/config directory structure             │
│   ├── scan      - Scan and cache configuration files                   │
│   ├── enable    - Enable config injection in recorder                 │
│   ├── disable   - Disable config injection                            │
│   ├── clear     - Remove config directory and cache                  │
│   └── status    - Show current configuration status                   │
│                                                                          │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        Config Cache                                   │
│   - cache.mcap file with all config as attachments                    │
│   - Attachment names preserve directory structure                    │
│   - Single read loads all config - no per-file I/O                  │
└──────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌──────────────────────────────────────────────────────────────────────┐
│                   AxonRecorder Integration                            │
│   - Reads cached config on recording start                           │
│   - Writes files as MCAP attachments                                 │
│   - Preserves directory structure in attachment names                │
└──────────────────────────────────────────────────────────────────────┘
```

## Commands

### 1. axon_config init

Create the `/axon/config` directory structure with default subdirectories.

```bash
axon_config init
```

**Behavior:**
- Creates `/axon/config` directory if it doesn't exist
- Creates default subdirectories:
  - `/axon/config/robot/` - Robot-specific configuration
  - `/axon/config/sensors/` - Sensor calibration files
  - `/axon/config/scene/` - Scene/environment configuration
- Prints created directories

**Output:**
```
Created /axon/config/
Created /axon/config/robot/
Created /axon/config/sensors/
Created /axon/config/scene/
```

### 2. axon_config scan

Scan `/axon/config` directory and generate persistent cache.

```bash
axon_config scan
```

**Behavior:**
- Recursively scans `/axon/config` directory
- Generates cache file at `/axon/config/cache.mcap`
- Reads all file contents and embeds them as attachments
- Records file metadata in attachment names (preserves directory structure)
- Prints directory structure and summary

**Output:**
```
Scanning /axon/config...
├── robot/
│   ├── robot_type.txt (1 KB)
│   ├── serial_number.txt (12 B)
│   └── urdf/ (2 files)
├── sensors/
│   ├── camera/
│   │   ├── intrinsics_left.json (2 KB)
│   │   └── intrinsics_right.json (2 KB)
│   └── lidar/
│       └── calibration.yaml (1 KB)
└── scene/
    └── warehouse_nav.json (512 B)

Cache written: 8 files, 6 KB
```

**Cache Format:**

All config files stored as MCAP attachments:

```
cache.mcap
├── Attachment: "robot/robot_type.txt"
│   └── Content: [file bytes]
├── Attachment: "robot/serial_number.txt"
│   └── Content: [file bytes]
├── Attachment: "sensors/camera/intrinsics_left.json"
│   └── Content: [file bytes]
└── ... (one attachment per config file)
```

### 3. axon_config enable

Enable configuration injection in recordings.

```bash
axon_config enable
```

**Behavior:**
- Creates marker file `/axon/config/.enabled`
- If cache doesn't exist, prints warning
- If config directory doesn't exist, prints warning

**Output:**
```
Config injection enabled.
Cache: /axon/config/cache.mcap (8 files, 6 KB)
```

**Warning Cases:**
```
Warning: Config cache not found. Run 'axon_config scan' first.
Config injection enabled, but no files will be written.
```

### 4. axon_config disable

Disable configuration injection.

```bash
axon_config disable
```

**Behavior:**
- Removes `/axon/config/.enabled` marker file
- AxonRecorder will skip config injection in future recordings

**Output:**
```
Config injection disabled.
```

### 5. axon_config clear

Remove configuration directory and cache.

```bash
axon_config clear
```

**Behavior:**
- Requires **interactive confirmation** (cannot be skipped)
- Shows summary of what will be deleted
- Supports `--force` flag for non-interactive use (scripting)
- Removes entire `/axon/config` directory
- Also implicitly disables config injection

**Interactive Confirmation:**
```
This will DELETE all configuration data:
  Directory: /axon/config/
  Files: 8
  Size: 6 KB

Are you sure? Type 'yes' to confirm: yes

Deleted /axon/config/ and all contents.
Config injection disabled.
```

**Aborted (user typed anything other than 'yes'):**
```
This will DELETE all configuration data:
  Directory: /axon/config/
  Files: 8
  Size: 6 KB

Are you sure? Type 'yes' to confirm: no

Operation cancelled. No changes made.
```

**Non-Interactive Mode (for scripts):**
```bash
axon_config clear --force
```

**Output:**
```
Deleted /axon/config/ and all contents.
Config injection disabled.
```

### 6. axon_config status

Show current configuration status.

```bash
axon_config status
```

**Output:**
```
Config Status: ENABLED
Config Directory: /axon/config
Cache File: /axon/config/cache.mcap
Files Cached: 8
Total Size: 6 KB
Last Scanned: 2025-02-08 10:30:00
```

## AxonRecorder Integration

### Recording Flow with Config Enabled

```
When recording starts:
1. Check if /axon/config/.enabled exists
2. If enabled, read /axon/config/cache.mcap
3. For each attachment in cache:
   - Copy to recording MCAP with "config/" prefix
4. Log summary of injected files
```

### Attachment Naming Scheme

```
Cache MCAP:                Recording MCAP:
├── "robot/type.txt"   →   ├── "config/robot/type.txt"
├── "sensors/camera/"   →   ├── "config/sensors/camera/"
└── ...                     └── "config/..."
```

Config files are prefixed with `config/` to avoid conflicts with other attachments.

## Error Handling

| Scenario | Behavior |
|----------|----------|
| `/axon/config` doesn't exist (scan) | Error: "Config directory not found. Run 'axon_config init' first." |
| Cache file missing (enable) | Warning: "Cache not found. Run 'axon_config scan' first." |
| Cache file invalid (recording) | Warning: "Cache corrupted. No config injected." |
| No attachments in cache | Warning: "Cache exists but has no attachments. No config injected." |
| Permission denied | Error: Print error message, exit with non-zero |
| `clear` confirmation timeout (5 min) | Warning: "Confirmation timed out. Operation cancelled." |
| `clear` cancelled by user | Info: "Operation cancelled. No changes made." |
| Non-interactive `clear` without `--force` | Error: "Cannot confirm in non-interactive mode. Use --force to bypass." |

## Incremental Cache Updates

### axon_config scan --incremental

Scan only changed files (based on mtime/size).

```bash
axon_config scan --incremental
```

**Behavior:**
1. Read existing cache MCAP
2. Compare each file's mtime/size with filesystem
3. Update only changed files:
   - **Added**: New files → append to cache
   - **Modified**: Changed files → replace attachment
   - **Deleted**: Removed files → drop from cache
4. Write new cache atomically (temp file + rename)

**Output:**
```
Incremental scan: 2 changed, 1 added, 0 removed
Updated: sensors/camera/intrinsics.json (modified)
Updated: robot/serial_number.txt (modified)
Added: scene/warehouse_2.json
Cache written: 9 files, 7 KB
```

### When to Use Incremental vs Full Scan

| Scenario | Recommended | Reason |
|----------|-------------|--------|
| Initial setup | Full scan | No existing cache |
| Small edits (1-5 files) | Incremental | Faster |
| Many changes (>50%) | Full scan | Simpler, comparable speed |
| After `git pull` | Full scan | Batch changes likely |
| Regular automation | Incremental | Efficiency |

## Future Enhancements (Out of Scope)

- `axon_config add <file>` - Add individual files to config
- `axon_config remove <file>` - Remove files from config
- `axon_config validate` - Validate cache integrity
- `axon_config inspect` - Inspect cache contents
- `axon_config export <tar.gz>` - Export config as archive
- `--config-dir` flag - Custom config directory location
- `--dry-run` flag - Preview what would be deleted without actually deleting
- Remote config fetching from server

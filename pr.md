## Summary

Optimize build scripts by migrating ROS1 build system from CMake to `catkin build`, adding code formatting targets, and applying code formatting across the codebase.

## Motivation

The ROS1 build system was using raw CMake commands, which is inconsistent with ROS best practices and doesn't handle catkin workspace dependencies properly. This change:
- Migrates ROS1 to use `catkin build` (standard catkin tool)
- Ensures consistent build approach between ROS1 (catkin) and ROS2 (colcon)
- Adds `make format` and `make lint` for code quality
- Improves code readability with automated formatting

## Changes

### Build System Improvements

**ROS1 Build Migration:**
- `cpp-build-ros1`: Use `catkin build` instead of raw `cmake` commands
- `clean-cpp`: Use `catkin clean --yes` for ROS1 cleanup
- `install`: Use `catkin build` for ROS1 installation

**Benefits:**
- Proper handling of catkin workspace dependencies
- Automatic parallel builds
- Better integration with catkin tools
- Consistent with ROS2 `colcon build` approach

### Code Quality Tools

- `make format` - Format all C/C++ and Rust code
- `make format-cpp` - Format C/C++ code using clang-format
- `make lint` - Check all code (cppcheck + clippy)
- `make lint-cpp` - Check C/C++ code using cppcheck

### Code Formatting

Applied `clang-format` to **60 files** including:
- cpp/axon_logging/
- cpp/axon_mcap/
- cpp/axon_uploader/
- ros/src/axon_recorder/src/

## Testing

- [x] ROS1 build with `catkin build` succeeds
- [x] ROS1 clean with `catkin clean` works
- [x] `make format` runs successfully
- [x] `make lint` executes without critical errors
- [x] Both ROS1 and ROS2 builds work correctly

## Backward Compatibility

No breaking changes to functionality. The build system transition is transparent to end users.

## Related Files

**Modified:**
- [Makefile](Makefile) - Migrate ROS1 to catkin build, add format/lint targets

**Formatted:** 60 files (see git diff for details)

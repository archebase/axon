# Axon Robot Adapter Interface

This directory contains the public interface used by robot adapter plugins.

Robot adapters are loaded by `axon-agent` with `dlopen()`. The shared object
must export three C symbols:

```cpp
extern "C" const axon::agent::RobotAdapterDescriptor* axon_agent_get_adapter_descriptor();
extern "C" axon::agent::RobotAdapter* axon_agent_create_adapter();
extern "C" void axon_agent_destroy_adapter(axon::agent::RobotAdapter*);
```

The plugin implementation should include:

```cpp
#include <axon/agent/robot_adapter.hpp>
```

and inherit from:

```cpp
axon::agent::RobotAdapter
```

## Lifecycle Model

`RobotAdapter::start()`, `stop()`, and `force_stop()` are adapter-specific
lifecycle hooks. Long-running robot programs should still run as child
processes managed by `axon-agent` ProcessManager. This keeps PID tracking,
process group stop, restart discovery, and log collection centralized.

Do not keep unstable third-party SDK loops inside the `axon-agent` process.
`try/catch` only handles C++ exceptions. It does not protect `axon-agent` from
segmentation faults, `abort()`, deadlocks, or memory corruption inside a plugin.

Use the adapter hooks for short operations such as:

- validating robot SDK availability
- generating launch configuration
- preparing environment files
- requesting graceful shutdown through a vendor control API
- reporting adapter-specific status

## Build A Plugin

Build the demo adapter:

```bash
./scripts/build_robot_adapter.sh \
  --file examples/demo_robot_adapter.cpp \
  --output /tmp/libaxon_demo_robot.so
```

Build a profile-local adapter for the bundled demo profile:

```bash
./scripts/build_robot_adapter.sh \
  --file examples/demo_robot_adapter.cpp \
  --output ../../apps/axon_agent/examples/robots/demo_robot/libaxon_demo_robot.so
```

Then start `axon-agent` with:

```bash
axon-agent \
  --robot-profile-path apps/axon_agent/examples/robots \
  --state-dir /tmp/axon-agent-state
```

## ABI Rules

- Keep `abi_version` equal to `axon::agent::kRobotAdapterAbiVersion`.
- Export all three required symbols.
- Create the adapter object in `axon_agent_create_adapter()`.
- Destroy the adapter object in `axon_agent_destroy_adapter()`.
- Do not let C++ exceptions escape adapter methods.
- Avoid passing ownership of memory across the plugin boundary.

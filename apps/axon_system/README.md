# axon-system

`axon-system` is the local Axon system monitor service. Phase 1 provides the
standalone service skeleton, basic status reporting, and graceful shutdown.
Phase 2 adds host CPU, memory, disk, and network metrics to `/rpc/state` and
`/rpc/metrics`.

## Build

```bash
cmake -S . -B build -DAXON_BUILD_TESTS=ON
cmake --build build --target axon_system test_system_service
```

## Smoke Test

```bash
./build/axon_system/axon-system --host 127.0.0.1 --port 8091 --state-dir /tmp/axon-system
curl http://127.0.0.1:8091/health
curl http://127.0.0.1:8091/rpc/state
curl http://127.0.0.1:8091/rpc/metrics
curl -X POST http://127.0.0.1:8091/rpc/quit
```

Metrics use bytes for memory, disk, and network counters; network rates use
bytes per second; CPU usage is reported as a percentage. The service
intentionally has no ROS or middleware dependencies.

# Axon Utils

Common utility library for the Axon project.

## Overview

This library contains middleware-independent utilities used across the Axon ecosystem:

- **State Machine Framework** - Thread-safe state management with transition guards
- **Recorder Service Interface** - Abstract interface for recorder service adapters
- **SPSC Queue** - Lock-free single-producer single-consumer queue
- **MPSC Queue** - Lock-free multi-producer single-consumer queue
- **Worker Thread Pool** - High-performance worker thread pool for message processing

Note: Common types (TaskConfig, RecorderConfig, etc.) have been moved to apps/axon_recorder.

## Usage

Link against this library in your CMakeLists.txt:

```cmake
target_link_libraries(your_target axon_utils)
```

## Components

### State Machine

Provides thread-safe state management:

```cpp
#include <axon_utils/state_machine.hpp>

using namespace axon::utils;

StateMachine<MyState> sm;
sm.transition(MyState::IDLE, MyState::READY, error);
```

### Lock-Free Queues

High-performance lock-free queues for zero-copy message transfer:

```cpp
#include <axon_utils/spsc_queue.hpp>

using namespace axon::utils;

SPSCQueue<Message> queue(4096);
queue.try_push(std::move(message));
Message msg;
if (queue.try_pop(msg)) {
  // Process message
}
```

### Worker Thread Pool

Manages worker threads for processing messages from per-topic queues:

```cpp
#include <axon_utils/worker_thread_pool.hpp>

using namespace axon::utils;

WorkerThreadPool pool;
pool.create_topic_worker("topic", handler);
pool.start();
pool.try_push("topic", MessageItem{timestamp, std::move(data)});
```

## Building

```bash
cd core/axon_utils
mkdir build && cd build
cmake ..
make
```

## Testing

```bash
make test
```

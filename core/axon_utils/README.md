# Axon Utils

Common utility library for the Axon project.

## Overview

This library contains middleware-independent utilities used across the Axon ecosystem:

- **State Machine Framework** - Thread-safe state management with transition guards
- **Configuration Parser** - YAML configuration file parsing
- **HTTP Client** - HTTP/HTTPS client for server callbacks
- **Common Types** - Shared data structures (TaskConfig, TopicConfig, etc.)

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

StateManager sm;
sm.transition_to(State::READY);
```

### Configuration Parser

Parse YAML configuration files:

```cpp
#include <axon_utils/config_parser.hpp>

using namespace axon::utils;

RecorderConfig config = RecorderConfig::from_yaml("config.yaml");
```

### HTTP Client

Make HTTP/HTTPS requests:

```cpp
#include <axon_utils/http_client.hpp>

using namespace axon::utils;

HttpClient client;
auto result = client.post(url, body, auth_token);
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

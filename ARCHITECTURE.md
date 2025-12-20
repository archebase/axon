# Full Implementation Architecture

## Overview

This document describes the complete, production-ready implementation of Axon by ArcheBase that replaces all simplified/placeholder code with full, high-performance implementations.

## Key Architectural Improvements

### 1. Message Introspection System

**Before**: Placeholder converters, no dynamic message handling
**After**: Complete introspection system using ROS message traits

- **MessageIntrospector**: Abstract interface for ROS1/ROS2 message introspection
- **Ros1MessageIntrospector**: Full ROS1 implementation using message_traits
- **Ros2MessageIntrospector**: Full ROS2 implementation using rosidl_typesupport
- **IntrospectionMessageConverter**: Dynamic converter using introspection

**Benefits**:
- Supports ALL ROS message types without hardcoding
- Zero-copy where possible
- Type-safe field access

### 2. High-Performance Batch Manager

**Before**: Simplified array concatenation, inefficient memory usage
**After**: Efficient message batching with direct MCAP serialization

**Key Features**:
- **Message Queues**: Per-topic lock-free queues for efficient batching
- **Direct Serialization**: Messages serialized directly to MCAP format
- **Lock-free Optimizations**: Atomic counters for approximate size
- **Efficient Batching**: Batch messages before writing to reduce I/O overhead
- **Proper Resource Management**: RAII, move semantics

**Performance Improvements**:
- Reduced CPU overhead (no schema conversion)
- Lower memory allocations
- Better cache locality

### 3. Schema Merging System

**Before**: Single schema per dataset, no multi-topic support
**After**: Full schema merging with conflict resolution

**Features**:
- **SchemaMerger**: Merges schemas from multiple topics
- **Field Prefixing**: Automatic prefixing to avoid conflicts
- **Type Compatibility**: Checks for compatible field types
- **Recording Schema**: Optimized schema with timestamp/topic fields

**Benefits**:
- Single dataset for multiple topics
- Efficient storage
- Easy querying across topics

### 4. Complete ROS Interface Implementation

**Before**: Placeholder subscriptions/services
**After**: Full typed subscriptions and services

**ROS1**:
- Proper `ros::Subscriber` creation
- Service advertisement with callbacks
- Message event handling

**ROS2**:
- Generic subscriptions using `rclcpp::create_generic_subscription`
- Generic services using `rclcpp::create_generic_service`
- Serialized message handling
- Version compatibility (Foxy+)

### 5. Memory Management

**Improvements**:
- **Message Buffers**: Efficient buffering for serialized messages
- **Smart Pointers**: Proper RAII throughout
- **Move Semantics**: Efficient resource transfer
- **Direct Storage**: Messages stored directly in MCAP format without conversion

### 6. Error Handling & Resilience

**Features**:
- Comprehensive error checking
- Graceful degradation
- Retry logic in writer thread
- Proper error propagation

## Performance Optimizations

### Direct Serialization Architecture

```
ROS Message → Serialization → MCAP Writer → Disk
     ↓              ↓              ↓
  (copy)      (direct)      (buffered)
```

### Lock-Free Optimizations

- Atomic counters for batch size
- Lock-free queue size approximation
- Minimal mutex contention

### Memory Efficiency

- Efficient message buffering
- Reusable serialization buffers
- Direct MCAP format storage (no intermediate conversions)

## Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  ROS Interface Layer                         │
│  ┌──────────────┐              ┌──────────────┐            │
│  │  ROS1 Impl   │              │  ROS2 Impl   │            │
│  └──────────────┘              └──────────────┘            │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│              Message Serialization Layer                     │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ Message Factory  │  │ Serialization           │        │
│  │ (Dynamic)        │  │ (ROS format)             │        │
│  └──────────────────┘  └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                  MCAP Writer Layer                           │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ MCAP Writer      │  │ Compression             │        │
│  │ (Thread-safe)    │  │ (Zstd/LZ4)              │        │
│  └──────────────────┘  └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

1. **Message Reception**: ROS callback receives message
2. **Serialization**: Message is serialized to ROS message format
3. **Batching**: Messages are queued for batch writing
4. **Flush**: When batch size/interval reached, write to MCAP
5. **Async Write**: Background thread writes to MCAP file
6. **MCAP Storage**: Messages written to MCAP file on disk

## Threading Model

- **Main Thread**: ROS callbacks, message serialization
- **Writer Thread**: Async MCAP file writing
- **Lock Strategy**: Minimal locking, thread-safe MCAP writer

## Memory Model

- **Message Queues**: Per-topic queues for batching
- **MCAP Buffers**: Efficient buffering for file writes
- **Compression**: Optional compression (Zstd/LZ4) reduces disk I/O

## Future Enhancements

1. **Advanced Compression**: Per-topic compression settings
2. **Indexing**: Fast topic/time-based queries
3. **Streaming**: Real-time streaming to remote storage
4. **Metrics**: Performance metrics and monitoring
5. **Multi-file**: Automatic file rotation for long recordings

## Migration Notes

The system has been migrated from Lance/Arrow to MCAP:
- ✅ MCAP Writer: Thread-safe MCAP file writing
- ✅ Message Serialization: Direct ROS message serialization
- ✅ ROS Interfaces: Complete subscription/service implementation
- ✅ Configuration: YAML-based configuration parsing
- ✅ Error Handling: Comprehensive throughout


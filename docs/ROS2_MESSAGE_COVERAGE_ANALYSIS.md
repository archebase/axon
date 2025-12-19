# ROS2 Message Coverage Analysis

## Overview

This document analyzes the current message type support in Axon's `cpp/` and `c/` directories compared to:
1. Standard ROS2 common messages
2. Foxglove SDK ROS2 schemas

## Current Implementation Status

### Architecture

The current implementation uses **dynamic message introspection** which should theoretically support any ROS2 message type. However, explicit registration and testing has been done for only a limited set.

### Currently Supported/Registered Messages

Based on `ros/axon_ros1/src/register_common_messages.cpp` (ROS1, but similar pattern expected for ROS2):

#### std_msgs
- ✅ `String`
- ✅ `Int32`
- ✅ `Float64`
- ✅ `Bool`
- ✅ `Header`

#### sensor_msgs
- ✅ `Image`
- ✅ `PointCloud2`

#### nav_msgs
- ✅ `Odometry`

#### geometry_msgs
- ✅ `Pose`
- ✅ `Twist`

### Schema Documentation

`config/message_schemas.yaml` documents schemas for:
- `sensor_msgs/Image`
- `sensor_msgs/PointCloud2`
- `nav_msgs/Odometry`
- `std_msgs/String`
- `std_msgs/Int32`
- `std_msgs/Float64`

## Standard ROS2 Common Messages (Potentially Missing)

### std_msgs (Standard Messages)
**Currently Supported**: String, Int32, Float64, Bool, Header

**Potentially Missing**:
- `Byte`, `Char`
- `Int8`, `Int16`, `Int64`
- `UInt8`, `UInt16`, `UInt32`, `UInt64`
- `Float32`
- `Time`, `Duration`
- `ColorRGBA`
- `MultiArrayLayout`, `MultiArrayDimension`, `MultiArray`

### sensor_msgs (Sensor Messages)
**Currently Supported**: Image, PointCloud2

**Potentially Missing**:
- `CompressedImage` ⚠️ (Very common)
- `LaserScan` ⚠️ (Very common)
- `Imu` ⚠️ (Very common)
- `NavSatFix` (GPS data)
- `JointState` (Robot joint states)
- `Temperature`
- `FluidPressure`
- `BatteryState`
- `CameraInfo`
- `Range` (Ultrasonic/sonar)

### geometry_msgs (Geometric Messages)
**Currently Supported**: Pose, Twist

**Potentially Missing**:
- `Point`, `Point32`, `PointStamped`
- `Quaternion`, `QuaternionStamped`
- `PoseStamped`, `PoseWithCovariance`, `PoseWithCovarianceStamped`
- `TwistStamped`, `TwistWithCovariance`, `TwistWithCovarianceStamped`
- `Transform`, `TransformStamped` ⚠️ (Very common for TF)
- `Vector3`, `Vector3Stamped`
- `Wrench`, `WrenchStamped`
- `Accel`, `AccelStamped`, `AccelWithCovariance`

### nav_msgs (Navigation Messages)
**Currently Supported**: Odometry

**Potentially Missing**:
- `Path` ⚠️ (Very common)
- `OccupancyGrid` ⚠️ (Very common for mapping)
- `MapMetaData`
- `GridCells`
- `GetMap` (service)

### tf2_msgs (TF2 Messages)
**Currently Missing**:
- `TFMessage` ⚠️ (Very common - transform tree)

### visualization_msgs (Visualization Messages)
**Currently Missing**:
- `Marker` ⚠️ (Common for visualization)
- `MarkerArray`
- `ImageMarker`

### diagnostic_msgs (Diagnostic Messages)
**Currently Missing**:
- `DiagnosticStatus`
- `DiagnosticArray`
- `KeyValue`

### action_msgs (Action Messages)
**Currently Missing**:
- `GoalStatus`
- `GoalStatusArray`

## Foxglove SDK ROS2 Schemas

The Foxglove SDK defines custom message types (not standard ROS2). These represent common data patterns:

### Foxglove Message Types
1. `ArrowPrimitive.msg`
2. `CameraCalibration.msg`
3. `CircleAnnotation.msg`
4. `Color.msg`
5. `CompressedImage.msg` (similar to `sensor_msgs/CompressedImage`)
6. `CompressedVideo.msg`
7. `CubePrimitive.msg`
8. `CylinderPrimitive.msg`
9. `FrameTransform.msg` (similar to `geometry_msgs/TransformStamped`)
10. `FrameTransforms.msg`
11. `GeoJSON.msg`
12. `Grid.msg` (similar to `nav_msgs/OccupancyGrid`)
13. `ImageAnnotations.msg`
14. `KeyValuePair.msg`
15. `LaserScan.msg` (similar to `sensor_msgs/LaserScan`)
16. `LocationFix.msg` (similar to `sensor_msgs/NavSatFix`)
17. `LocationFixes.msg`
18. `Log.msg`
19. `ModelPrimitive.msg`
20. `PackedElementField.msg`
21. `Point2.msg`
22. `PointCloud.msg` (similar to `sensor_msgs/PointCloud2`)
23. `PointsAnnotation.msg`
24. `PoseInFrame.msg`
25. `PosesInFrame.msg`
26. `RawAudio.msg`
27. `RawImage.msg` (similar to `sensor_msgs/Image`)
28. `SceneEntity.msg`
29. `SceneEntityDeletion.msg`
30. `SceneUpdate.msg`
31. `SpherePrimitive.msg`
32. `TextAnnotation.msg`
33. `TextPrimitive.msg`
34. `TriangleListPrimitive.msg`
35. `Vector2.msg`
36. `VoxelGrid.msg`

**Note**: These are Foxglove-specific message types, not standard ROS2 messages. However, they represent common data patterns that might be useful for reference.

## Critical Gaps Analysis

### High Priority (Very Common in ROS2 Applications)

1. **`sensor_msgs/CompressedImage`** - Very common for camera data
2. **`sensor_msgs/LaserScan`** - Essential for LIDAR/scan data
3. **`sensor_msgs/Imu`** - Essential for IMU data
4. **`geometry_msgs/TransformStamped`** - Critical for TF (transform tree)
5. **`tf2_msgs/TFMessage`** - TF2 transform messages
6. **`nav_msgs/Path`** - Common for path planning
7. **`nav_msgs/OccupancyGrid`** - Essential for mapping
8. **`geometry_msgs/PoseStamped`** - Common pose with timestamp
9. **`geometry_msgs/TwistStamped`** - Common velocity with timestamp

### Medium Priority

1. **`sensor_msgs/NavSatFix`** - GPS/GNSS data
2. **`sensor_msgs/JointState`** - Robot joint states
3. **`sensor_msgs/CameraInfo`** - Camera calibration
4. **`visualization_msgs/Marker`** - Visualization markers
5. **`std_msgs` numeric types** - Int8, Int16, Int64, UInt8, etc.

### Low Priority

1. Other specialized sensor messages
2. Diagnostic messages
3. Action messages

## Recommendations

### 1. Verify Dynamic Introspection Works for All Message Types

The current implementation uses dynamic introspection (`MessageIntrospector`), which should handle any ROS2 message type. However, you should:

- **Test** with common message types to ensure they work correctly
- **Document** which message types have been tested
- **Add unit tests** for common message types

### 2. Add Explicit Registration for Common Messages (Optional)

While not strictly necessary (due to dynamic introspection), explicit registration can:
- Provide compile-time type safety
- Enable optimizations for specific message types
- Serve as documentation of supported types

Consider creating `ros/axon_ros2/src/register_common_messages.cpp` similar to ROS1 version.

### 3. Add Schema Documentation

Update `config/message_schemas.yaml` to document schemas for:
- High-priority missing messages
- Any message types you plan to optimize

### 4. Testing Strategy

Create integration tests that verify:
- Message introspection works for all standard ROS2 message types
- Arrow schema generation is correct
- Data conversion preserves all fields
- Lance format writing works correctly

## Implementation Notes

### Current Architecture

```
cpp/src/core/
├── message_introspection.hpp/cpp  # Dynamic introspection (should handle any message)
├── message_converter.hpp/cpp      # Generic converter using introspection
└── ...

ros/axon_ros2/src/
├── ros2_introspection.cpp         # ROS2-specific introspection implementation
└── ros2_interface.cpp             # ROS2 interface (uses generic subscriptions)
```

### Key Insight

The **dynamic introspection approach** means you don't need explicit registration for each message type. However, you should:

1. **Verify** it works correctly for complex nested messages
2. **Test** with high-priority message types
3. **Document** limitations (if any)
4. **Consider optimizations** for frequently used message types

## Conclusion

**Good News**: The dynamic introspection architecture should support most ROS2 message types without explicit registration.

**Action Items**:
1. ✅ Verify introspection works for high-priority missing messages
2. ✅ Add tests for common message types (CompressedImage, LaserScan, Imu, TransformStamped, etc.)
3. ✅ Update documentation with tested message types
4. ⚠️ Consider explicit registration/optimization for very common types if performance is an issue

The current implementation appears to be well-architected for handling arbitrary ROS2 message types, but explicit testing and documentation would be valuable.







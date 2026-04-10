// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/schema_resolver.hpp"

namespace fs = std::filesystem;
using axon::recorder::SchemaResolver;

// =============================================================================
// Test Fixture: Creates temporary .msg files for testing
// =============================================================================

class SchemaResolverTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create temp directory structure
    test_dir_ = fs::temp_directory_path() / "axon_schema_test";
    fs::remove_all(test_dir_);
    fs::create_directories(test_dir_);

    share_path_ = test_dir_ / "share";

    // Create standard ROS2 message packages
    create_msg_file("std_msgs", "Header",
      "# Standard header\n"
      "builtin_interfaces/Time stamp\n"
      "string frame_id");

    create_msg_file("builtin_interfaces", "Time",
      "# Time representation\n"
      "int32 sec\n"
      "uint32 nanosec");

    create_msg_file("builtin_interfaces", "Duration",
      "int32 sec\n"
      "uint32 nanosec");

    create_msg_file("sensor_msgs", "Image",
      "# Image message\n"
      "std_msgs/Header header\n"
      "uint32 height\n"
      "uint32 width\n"
      "string encoding\n"
      "uint8 is_bigendian\n"
      "uint32 step\n"
      "uint8[] data");

    create_msg_file("sensor_msgs", "Imu",
      "std_msgs/Header header\n"
      "geometry_msgs/Quaternion orientation\n"
      "float64[9] orientation_covariance\n"
      "geometry_msgs/Vector3 angular_velocity\n"
      "float64[9] angular_velocity_covariance\n"
      "geometry_msgs/Vector3 linear_acceleration\n"
      "float64[9] linear_acceleration_covariance");

    create_msg_file("geometry_msgs", "Quaternion",
      "float64 x 0\n"
      "float64 y 0\n"
      "float64 z 0\n"
      "float64 w 1");

    create_msg_file("geometry_msgs", "Vector3",
      "float64 x\n"
      "float64 y\n"
      "float64 z");

    // A message with constants (should not produce dependencies)
    create_msg_file("sensor_msgs", "PointField",
      "uint8 INT8=1\n"
      "uint8 UINT8=2\n"
      "uint8 INT16=3\n"
      "string name\n"
      "uint32 offset\n"
      "uint8 datatype\n"
      "uint32 count");

    // A message with bare type reference (same package)
    create_msg_file("custom_msgs", "Compound",
      "SubType sub\n"
      "uint32 value");

    create_msg_file("custom_msgs", "SubType",
      "string name\n"
      "float64 data");
  }

  void TearDown() override {
    fs::remove_all(test_dir_);
  }

  void create_msg_file(
    const std::string& package, const std::string& type, const std::string& content
  ) {
    fs::path msg_dir = share_path_ / package / "msg";
    fs::create_directories(msg_dir);
    std::ofstream file(msg_dir / (type + ".msg"));
    file << content;
  }

  fs::path test_dir_;
  fs::path share_path_;
};

// =============================================================================
// Basic Resolution Tests
// =============================================================================

TEST_F(SchemaResolverTest, ResolveSimplePrimitiveOnlyMessage) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("geometry_msgs/msg/Vector3");
  ASSERT_FALSE(result.empty());

  // Should contain the Vector3 definition without any dependencies
  EXPECT_NE(result.find("float64 x"), std::string::npos);
  EXPECT_NE(result.find("float64 y"), std::string::npos);
  EXPECT_NE(result.find("float64 z"), std::string::npos);

  // Should not have any separator (no dependencies)
  EXPECT_EQ(result.find("===="), std::string::npos);
}

TEST_F(SchemaResolverTest, ResolveMessageWithDependencies) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("sensor_msgs/msg/Image");
  ASSERT_FALSE(result.empty());

  // Top-level definition should appear first
  EXPECT_NE(result.find("uint32 height"), std::string::npos);
  EXPECT_NE(result.find("uint8[] data"), std::string::npos);

  // Should contain separator and MSG: lines for dependencies
  EXPECT_NE(result.find(std::string(80, '=')), std::string::npos);
  EXPECT_NE(result.find("MSG: std_msgs/msg/Header"), std::string::npos);
  EXPECT_NE(result.find("MSG: builtin_interfaces/msg/Time"), std::string::npos);

  // Dependency content should be present
  EXPECT_NE(result.find("string frame_id"), std::string::npos);
  EXPECT_NE(result.find("int32 sec"), std::string::npos);
  EXPECT_NE(result.find("uint32 nanosec"), std::string::npos);
}

TEST_F(SchemaResolverTest, ResolveDeepDependencyChain) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("sensor_msgs/msg/Imu");
  ASSERT_FALSE(result.empty());

  // Should contain all transitive dependencies
  EXPECT_NE(result.find("MSG: std_msgs/msg/Header"), std::string::npos);
  EXPECT_NE(result.find("MSG: builtin_interfaces/msg/Time"), std::string::npos);
  EXPECT_NE(result.find("MSG: geometry_msgs/msg/Quaternion"), std::string::npos);
  EXPECT_NE(result.find("MSG: geometry_msgs/msg/Vector3"), std::string::npos);
}

// =============================================================================
// ROS1/ROS2 Naming Convention Tests
// =============================================================================

TEST_F(SchemaResolverTest, ResolveRos1StyleName) {
  SchemaResolver resolver({share_path_.string()});

  // ROS1 style: "sensor_msgs/Image" (no /msg/)
  std::string result = resolver.resolve("sensor_msgs/Image");
  ASSERT_FALSE(result.empty());
  EXPECT_NE(result.find("uint32 height"), std::string::npos);
}

TEST_F(SchemaResolverTest, ResolveRos2StyleName) {
  SchemaResolver resolver({share_path_.string()});

  // ROS2 style: "sensor_msgs/msg/Image"
  std::string result = resolver.resolve("sensor_msgs/msg/Image");
  ASSERT_FALSE(result.empty());
  EXPECT_NE(result.find("uint32 height"), std::string::npos);
}

// =============================================================================
// ROS1 Built-in Type Mapping Tests
// =============================================================================

TEST_F(SchemaResolverTest, Ros1BuiltinTimeMapping) {
  // Create a message that uses ROS1-style "time" type
  create_msg_file("test_msgs", "WithTime",
    "time stamp\n"
    "string data");

  SchemaResolver resolver({share_path_.string()});
  std::string result = resolver.resolve("test_msgs/msg/WithTime");
  ASSERT_FALSE(result.empty());

  // Should map "time" to builtin_interfaces/msg/Time
  EXPECT_NE(result.find("MSG: builtin_interfaces/msg/Time"), std::string::npos);
  EXPECT_NE(result.find("int32 sec"), std::string::npos);
}

TEST_F(SchemaResolverTest, Ros1BuiltinHeaderMapping) {
  // Create a message that uses bare "Header" (ROS1 convention)
  create_msg_file("test_msgs", "WithHeader",
    "Header header\n"
    "string data");

  SchemaResolver resolver({share_path_.string()});
  std::string result = resolver.resolve("test_msgs/msg/WithHeader");
  ASSERT_FALSE(result.empty());

  // Should map "Header" to std_msgs/msg/Header
  EXPECT_NE(result.find("MSG: std_msgs/msg/Header"), std::string::npos);
}

// =============================================================================
// Bare Type Reference Tests (Same Package)
// =============================================================================

TEST_F(SchemaResolverTest, ResolveBareTypeInSamePackage) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("custom_msgs/msg/Compound");
  ASSERT_FALSE(result.empty());

  // Should resolve "SubType" as "custom_msgs/msg/SubType"
  EXPECT_NE(result.find("MSG: custom_msgs/msg/SubType"), std::string::npos);
  EXPECT_NE(result.find("string name"), std::string::npos);
}

// =============================================================================
// Constants and Comments Tests
// =============================================================================

TEST_F(SchemaResolverTest, ConstantsDoNotProduceDependencies) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("sensor_msgs/msg/PointField");
  ASSERT_FALSE(result.empty());

  // Should only contain primitives, no dependencies
  EXPECT_EQ(result.find("===="), std::string::npos);
  EXPECT_EQ(result.find("MSG:"), std::string::npos);
}

// =============================================================================
// Error Handling Tests
// =============================================================================

TEST_F(SchemaResolverTest, EmptySearchPaths) {
  SchemaResolver resolver({});

  std::string result = resolver.resolve("sensor_msgs/msg/Image");
  EXPECT_TRUE(result.empty());
  EXPECT_FALSE(resolver.get_last_error().empty());
}

TEST_F(SchemaResolverTest, NonExistentType) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("nonexistent_pkg/msg/NoSuchType");
  EXPECT_TRUE(result.empty());
  EXPECT_FALSE(resolver.get_last_error().empty());
}

TEST_F(SchemaResolverTest, InvalidTypeName) {
  SchemaResolver resolver({share_path_.string()});

  std::string result = resolver.resolve("");
  EXPECT_TRUE(result.empty());
}

// =============================================================================
// Cache Tests
// =============================================================================

TEST_F(SchemaResolverTest, CachingWorks) {
  SchemaResolver resolver({share_path_.string()});

  // First resolution
  std::string result1 = resolver.resolve("sensor_msgs/msg/Image");
  ASSERT_FALSE(result1.empty());

  // Second resolution should return same result (from cache)
  std::string result2 = resolver.resolve("sensor_msgs/msg/Image");
  EXPECT_EQ(result1, result2);
}

TEST_F(SchemaResolverTest, ClearCacheWorks) {
  SchemaResolver resolver({share_path_.string()});

  std::string result1 = resolver.resolve("sensor_msgs/msg/Image");
  ASSERT_FALSE(result1.empty());

  resolver.clear_cache();

  // Should still work after clearing cache
  std::string result2 = resolver.resolve("sensor_msgs/msg/Image");
  EXPECT_EQ(result1, result2);
}

// =============================================================================
// Multiple Search Paths Tests
// =============================================================================

TEST_F(SchemaResolverTest, MultipleSearchPaths) {
  // Create a second share path with a custom message
  fs::path share_path2 = test_dir_ / "share2";
  fs::path msg_dir = share_path2 / "custom_pkg" / "msg";
  fs::create_directories(msg_dir);
  {
    std::ofstream file(msg_dir / "MyMsg.msg");
    file << "uint32 value\nstring name";
  }

  SchemaResolver resolver({share_path_.string(), share_path2.string()});

  // Should find standard messages in first path
  std::string result1 = resolver.resolve("sensor_msgs/msg/Image");
  EXPECT_FALSE(result1.empty());

  // Should find custom message in second path
  std::string result2 = resolver.resolve("custom_pkg/msg/MyMsg");
  EXPECT_FALSE(result2.empty());
  EXPECT_NE(result2.find("uint32 value"), std::string::npos);
}

// =============================================================================
// Array Type Tests
// =============================================================================

TEST_F(SchemaResolverTest, ArrayTypeDependencyResolved) {
  // Create a message with array of complex type
  create_msg_file("test_msgs", "WithArray",
    "geometry_msgs/Vector3[] points\n"
    "uint32 count");

  SchemaResolver resolver({share_path_.string()});
  std::string result = resolver.resolve("test_msgs/msg/WithArray");
  ASSERT_FALSE(result.empty());

  // Should resolve the array element type
  EXPECT_NE(result.find("MSG: geometry_msgs/msg/Vector3"), std::string::npos);
}

TEST_F(SchemaResolverTest, FixedSizeArrayDependencyResolved) {
  // Create a message with fixed-size array of complex type
  create_msg_file("test_msgs", "WithFixedArray",
    "geometry_msgs/Quaternion[4] orientations");

  SchemaResolver resolver({share_path_.string()});
  std::string result = resolver.resolve("test_msgs/msg/WithFixedArray");
  ASSERT_FALSE(result.empty());

  EXPECT_NE(result.find("MSG: geometry_msgs/msg/Quaternion"), std::string::npos);
}

// =============================================================================
// Isolated Colcon Install Layout Tests
// =============================================================================

TEST_F(SchemaResolverTest, IsolatedInstallLayout) {
  // Create isolated install directory structure:
  //   install/my_pkg/share/my_pkg/msg/MyMsg.msg
  fs::path install_dir = test_dir_ / "isolated_install";
  fs::path msg_dir = install_dir / "my_pkg" / "share" / "my_pkg" / "msg";
  fs::create_directories(msg_dir);
  {
    std::ofstream file(msg_dir / "MyMsg.msg");
    file << "uint32 value\nstring name";
  }

  // Point search path to the install root
  SchemaResolver resolver({install_dir.string()});

  std::string result = resolver.resolve("my_pkg/msg/MyMsg");
  ASSERT_FALSE(result.empty());
  EXPECT_NE(result.find("uint32 value"), std::string::npos);
  EXPECT_NE(result.find("string name"), std::string::npos);
}

TEST_F(SchemaResolverTest, IsolatedInstallWithDependencies) {
  // Simulate isolated install with two packages
  fs::path install_dir = test_dir_ / "isolated_install2";

  // Package A depends on Package B
  {
    fs::path dir = install_dir / "pkg_a" / "share" / "pkg_a" / "msg";
    fs::create_directories(dir);
    std::ofstream file(dir / "MsgA.msg");
    file << "pkg_b/MsgB sub\nuint32 count";
  }
  {
    fs::path dir = install_dir / "pkg_b" / "share" / "pkg_b" / "msg";
    fs::create_directories(dir);
    std::ofstream file(dir / "MsgB.msg");
    file << "float64 data\nstring label";
  }

  SchemaResolver resolver({install_dir.string()});

  std::string result = resolver.resolve("pkg_a/msg/MsgA");
  ASSERT_FALSE(result.empty());
  EXPECT_NE(result.find("uint32 count"), std::string::npos);
  EXPECT_NE(result.find("MSG: pkg_b/msg/MsgB"), std::string::npos);
  EXPECT_NE(result.find("float64 data"), std::string::npos);
}

TEST_F(SchemaResolverTest, MergedInstallWithSharePrefix) {
  // Simulate merged install with share/ prefix:
  //   install/share/my_pkg/msg/MyMsg.msg
  fs::path install_dir = test_dir_ / "merged_install";
  fs::path msg_dir = install_dir / "share" / "my_pkg2" / "msg";
  fs::create_directories(msg_dir);
  {
    std::ofstream file(msg_dir / "MyMsg2.msg");
    file << "int64 timestamp\nbool active";
  }

  SchemaResolver resolver({install_dir.string()});

  std::string result = resolver.resolve("my_pkg2/msg/MyMsg2");
  ASSERT_FALSE(result.empty());
  EXPECT_NE(result.find("int64 timestamp"), std::string::npos);
  EXPECT_NE(result.find("bool active"), std::string::npos);
}

TEST_F(SchemaResolverTest, MixedInstallLayouts) {
  // Test that both layouts work together in different search paths
  fs::path isolated_dir = test_dir_ / "mixed_isolated";
  fs::path merged_dir = test_dir_ / "mixed_merged";

  // Isolated: install/pkg_iso/share/pkg_iso/msg/IsoMsg.msg
  {
    fs::path dir = isolated_dir / "pkg_iso" / "share" / "pkg_iso" / "msg";
    fs::create_directories(dir);
    std::ofstream file(dir / "IsoMsg.msg");
    file << "uint8 iso_field";
  }

  // Merged: share/pkg_merged/msg/MergedMsg.msg
  {
    fs::path dir = merged_dir / "pkg_merged" / "msg";
    fs::create_directories(dir);
    std::ofstream file(dir / "MergedMsg.msg");
    file << "uint16 merged_field";
  }

  SchemaResolver resolver({isolated_dir.string(), merged_dir.string()});

  std::string r1 = resolver.resolve("pkg_iso/msg/IsoMsg");
  ASSERT_FALSE(r1.empty());
  EXPECT_NE(r1.find("uint8 iso_field"), std::string::npos);

  std::string r2 = resolver.resolve("pkg_merged/msg/MergedMsg");
  ASSERT_FALSE(r2.empty());
  EXPECT_NE(r2.find("uint16 merged_field"), std::string::npos);
}

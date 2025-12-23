/**
 * @file test_ros_introspection.cpp
 * @brief Unit tests for ros_introspection.cpp
 *
 * Tests the MessageIntrospectorFactory and message type parsing logic.
 * These tests do not require ROS - they test pure logic.
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "ros_introspection.hpp"

using namespace axon::core;
using namespace axon::recorder;

// ============================================================================
// MessageIntrospectorFactory Tests
// ============================================================================

class MessageIntrospectorFactoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Clear any previously set factory
    MessageIntrospectorFactory::set_factory(nullptr);
  }

  void TearDown() override {
    // Reset factory after each test
    MessageIntrospectorFactory::set_factory(nullptr);
  }
};

TEST_F(MessageIntrospectorFactoryTest, HasFactory_InitiallyFalse) {
  EXPECT_FALSE(MessageIntrospectorFactory::has_factory());
}

TEST_F(MessageIntrospectorFactoryTest, Create_ReturnsNullptrWhenNoFactory) {
  auto introspector = MessageIntrospectorFactory::create();
  EXPECT_EQ(introspector, nullptr);
}

TEST_F(MessageIntrospectorFactoryTest, SetFactory_MakesHasFactoryTrue) {
  // Create a simple mock introspector
  class MockIntrospector : public MessageIntrospector {
  public:
    bool get_message_descriptor(const std::string&, MessageTypeDescriptor&) override {
      return true;
    }
    bool get_field_value(const void*, const std::string&, void*) override {
      return false;
    }
    const void* get_field_pointer(const void*, const std::string&, size_t&) override {
      return nullptr;
    }
  };

  MessageIntrospectorFactory::set_factory([]() {
    return std::make_unique<MockIntrospector>();
  });

  EXPECT_TRUE(MessageIntrospectorFactory::has_factory());
}

TEST_F(MessageIntrospectorFactoryTest, Create_ReturnsInstanceWhenFactorySet) {
  class MockIntrospector : public MessageIntrospector {
  public:
    bool get_message_descriptor(const std::string&, MessageTypeDescriptor&) override {
      return true;
    }
    bool get_field_value(const void*, const std::string&, void*) override {
      return false;
    }
    const void* get_field_pointer(const void*, const std::string&, size_t&) override {
      return nullptr;
    }
  };

  MessageIntrospectorFactory::set_factory([]() {
    return std::make_unique<MockIntrospector>();
  });

  auto introspector = MessageIntrospectorFactory::create();
  EXPECT_NE(introspector, nullptr);
}

TEST_F(MessageIntrospectorFactoryTest, SetFactory_NullClearsFactory) {
  class MockIntrospector : public MessageIntrospector {
  public:
    bool get_message_descriptor(const std::string&, MessageTypeDescriptor&) override {
      return true;
    }
    bool get_field_value(const void*, const std::string&, void*) override {
      return false;
    }
    const void* get_field_pointer(const void*, const std::string&, size_t&) override {
      return nullptr;
    }
  };

  // Set a factory
  MessageIntrospectorFactory::set_factory([]() {
    return std::make_unique<MockIntrospector>();
  });
  EXPECT_TRUE(MessageIntrospectorFactory::has_factory());

  // Clear it with nullptr
  MessageIntrospectorFactory::set_factory(nullptr);
  EXPECT_FALSE(MessageIntrospectorFactory::has_factory());
}

// ============================================================================
// RosIntrospectorFactory Tests
// ============================================================================

class RosIntrospectorFactoryTest : public ::testing::Test {};

TEST_F(RosIntrospectorFactoryTest, Create_ReturnsNonNull) {
  auto introspector = RosIntrospectorFactory::create();
  EXPECT_NE(introspector, nullptr);
}

TEST_F(RosIntrospectorFactoryTest, Create_ReturnsNewInstanceEachTime) {
  auto introspector1 = RosIntrospectorFactory::create();
  auto introspector2 = RosIntrospectorFactory::create();
  
  EXPECT_NE(introspector1, nullptr);
  EXPECT_NE(introspector2, nullptr);
  EXPECT_NE(introspector1.get(), introspector2.get());
}

// ============================================================================
// MessageTypeDescriptor Parsing Tests
// ============================================================================

class MessageDescriptorTest : public ::testing::Test {
protected:
  std::unique_ptr<MessageIntrospector> introspector_;

  void SetUp() override {
    introspector_ = RosIntrospectorFactory::create();
  }
};

TEST_F(MessageDescriptorTest, GetMessageDescriptor_ROS2Style) {
  // ROS2 style: package/msg/MessageName
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("sensor_msgs/msg/Image", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.full_name, "sensor_msgs/msg/Image");
  EXPECT_EQ(descriptor.package, "sensor_msgs");
  EXPECT_EQ(descriptor.name, "Image");
}

TEST_F(MessageDescriptorTest, GetMessageDescriptor_ROS1Style) {
  // ROS1 style: package/MessageName
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("sensor_msgs/Image", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.full_name, "sensor_msgs/Image");
  EXPECT_EQ(descriptor.package, "sensor_msgs");
  EXPECT_EQ(descriptor.name, "Image");
}

TEST_F(MessageDescriptorTest, GetMessageDescriptor_NoPackage) {
  // Just message name, no package
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("Image", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.full_name, "Image");
  EXPECT_TRUE(descriptor.package.empty());
  EXPECT_EQ(descriptor.name, "Image");
}

TEST_F(MessageDescriptorTest, GetMessageDescriptor_StdMsgs) {
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("std_msgs/msg/Header", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.package, "std_msgs");
  EXPECT_EQ(descriptor.name, "Header");
}

TEST_F(MessageDescriptorTest, GetMessageDescriptor_GeometryMsgs) {
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("geometry_msgs/msg/Twist", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.package, "geometry_msgs");
  EXPECT_EQ(descriptor.name, "Twist");
}

TEST_F(MessageDescriptorTest, GetMessageDescriptor_NavMsgs) {
  MessageTypeDescriptor descriptor;
  bool result = introspector_->get_message_descriptor("nav_msgs/msg/Odometry", descriptor);

  EXPECT_TRUE(result);
  EXPECT_EQ(descriptor.package, "nav_msgs");
  EXPECT_EQ(descriptor.name, "Odometry");
}

// ============================================================================
// GetFieldPointer Tests
// ============================================================================

TEST_F(MessageDescriptorTest, GetFieldPointer_NullMessage_ReturnsNull) {
  size_t size = 0;
  const void* ptr = introspector_->get_field_pointer(nullptr, "data", size);
  
  EXPECT_EQ(ptr, nullptr);
}

// ============================================================================
// GetFieldValue Tests
// ============================================================================

TEST_F(MessageDescriptorTest, GetFieldValue_ReturnsFalse) {
  // The base implementation returns false (not implemented)
  int output = 0;
  bool result = introspector_->get_field_value(nullptr, "field", &output);
  
  EXPECT_FALSE(result);
}

// ============================================================================
// FieldDescriptor Default Values
// ============================================================================

TEST(FieldDescriptorTest, DefaultConstructor) {
  FieldDescriptor field;

  EXPECT_TRUE(field.name.empty());
  EXPECT_TRUE(field.type_name.empty());
  EXPECT_FALSE(field.is_array);
  EXPECT_FALSE(field.is_fixed_size);
  EXPECT_EQ(field.array_size, 0u);
  EXPECT_FALSE(field.is_builtin);
}

// ============================================================================
// MessageTypeDescriptor Default Values
// ============================================================================

TEST(MessageTypeDescriptorTest, DefaultConstructor) {
  MessageTypeDescriptor descriptor;

  EXPECT_TRUE(descriptor.full_name.empty());
  EXPECT_TRUE(descriptor.package.empty());
  EXPECT_TRUE(descriptor.name.empty());
  EXPECT_TRUE(descriptor.fields.empty());
  EXPECT_EQ(descriptor.size_bytes, 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


/**
 * @file test_message_factory.cpp
 * @brief Unit tests for MessageFactory
 *
 * Note: This test requires ROS to be available. It is conditionally compiled
 * only when building with ROS integration (AXON_ROS1 or AXON_ROS2 defined).
 */

#include <gtest/gtest.h>

// Only compile tests when ROS is available
#if defined(AXON_ROS1) || defined(AXON_ROS2)

#include "message_factory.hpp"

#if defined(AXON_ROS1)
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#elif defined(AXON_ROS2)
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#endif

using namespace axon::recorder;

// ============================================================================
// MessageFactory Tests
// ============================================================================

class MessageFactoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Register common message types for testing
#if defined(AXON_ROS1)
    MessageFactory::register_message_type<std_msgs::String>("std_msgs/String");
    MessageFactory::register_message_type<std_msgs::Int32>("std_msgs/Int32");
    MessageFactory::register_message_type<geometry_msgs::Twist>("geometry_msgs/Twist");
#elif defined(AXON_ROS2)
    MessageFactory::register_message_type<std_msgs::msg::String>("std_msgs/msg/String");
    MessageFactory::register_message_type<std_msgs::msg::Int32>("std_msgs/msg/Int32");
    MessageFactory::register_message_type<geometry_msgs::msg::Twist>("geometry_msgs/msg/Twist");
#endif
  }

  void TearDown() override {}
};

TEST_F(MessageFactoryTest, RegisterMessageType) {
#if defined(AXON_ROS1)
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/String"));
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/Int32"));
  EXPECT_TRUE(MessageFactory::is_registered("geometry_msgs/Twist"));
#elif defined(AXON_ROS2)
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/msg/String"));
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/msg/Int32"));
  EXPECT_TRUE(MessageFactory::is_registered("geometry_msgs/msg/Twist"));
#endif
}

TEST_F(MessageFactoryTest, UnregisteredType) {
  EXPECT_FALSE(MessageFactory::is_registered("nonexistent/Type"));
  EXPECT_FALSE(MessageFactory::is_registered(""));
}

TEST_F(MessageFactoryTest, CreateMessage) {
#if defined(AXON_ROS1)
  auto msg = MessageFactory::create_message("std_msgs/String");
  ASSERT_NE(msg, nullptr);

  // Cast and verify
  auto* typed_msg = static_cast<std_msgs::String*>(msg.get());
  EXPECT_TRUE(typed_msg->data.empty());
#elif defined(AXON_ROS2)
  auto msg = MessageFactory::create_message("std_msgs/msg/String");
  ASSERT_NE(msg, nullptr);

  auto* typed_msg = static_cast<std_msgs::msg::String*>(msg.get());
  EXPECT_TRUE(typed_msg->data.empty());
#endif
}

TEST_F(MessageFactoryTest, CreateNonexistentMessage) {
  auto msg = MessageFactory::create_message("nonexistent/Type");
  EXPECT_EQ(msg, nullptr);
}

TEST_F(MessageFactoryTest, GetMessageInfo) {
#if defined(AXON_ROS1)
  MessageFactory::MessageInfo info;
  EXPECT_TRUE(MessageFactory::get_message_info("std_msgs/String", info));
  EXPECT_EQ(info.datatype, "std_msgs/String");
  EXPECT_FALSE(info.definition.empty());
  EXPECT_FALSE(info.md5sum.empty());
#elif defined(AXON_ROS2)
  MessageFactory::MessageInfo info;
  EXPECT_TRUE(MessageFactory::get_message_info("std_msgs/msg/String", info));
  EXPECT_EQ(info.type_name, "std_msgs/msg/String");
#endif
}

TEST_F(MessageFactoryTest, GetMessageInfoNonexistent) {
  MessageFactory::MessageInfo info;
  EXPECT_FALSE(MessageFactory::get_message_info("nonexistent/Type", info));
}

#if defined(AXON_ROS1)
TEST_F(MessageFactoryTest, DeserializeMessageROS1) {
  // Create a serialized message
  std_msgs::String original;
  original.data = "Hello, World!";

  // Serialize
  uint32_t serial_size = ros::serialization::serializationLength(original);
  std::vector<uint8_t> buffer(serial_size);
  ros::serialization::OStream stream(buffer.data(), serial_size);
  ros::serialization::serialize(stream, original);

  // Deserialize using factory
  auto msg = MessageFactory::create_message("std_msgs/String");
  ASSERT_NE(msg, nullptr);

  EXPECT_TRUE(
    MessageFactory::deserialize_message("std_msgs/String", buffer.data(), serial_size, msg.get())
  );

  auto* deserialized = static_cast<std_msgs::String*>(msg.get());
  EXPECT_EQ(deserialized->data, "Hello, World!");
}
#endif

TEST_F(MessageFactoryTest, DeserializeNonexistentType) {
#if defined(AXON_ROS1)
  std::vector<uint8_t> buffer = {0x01, 0x02, 0x03};
  int dummy;
  EXPECT_FALSE(
    MessageFactory::deserialize_message("nonexistent/Type", buffer.data(), buffer.size(), &dummy)
  );
#endif
}

// ============================================================================
// Multiple Message Type Registration
// ============================================================================

TEST_F(MessageFactoryTest, RegisterMultipleTypes) {
  // Types already registered in SetUp
#if defined(AXON_ROS1)
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/String"));
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/Int32"));
  EXPECT_TRUE(MessageFactory::is_registered("geometry_msgs/Twist"));
#elif defined(AXON_ROS2)
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/msg/String"));
  EXPECT_TRUE(MessageFactory::is_registered("std_msgs/msg/Int32"));
  EXPECT_TRUE(MessageFactory::is_registered("geometry_msgs/msg/Twist"));
#endif
}

TEST_F(MessageFactoryTest, CreateDifferentTypes) {
#if defined(AXON_ROS1)
  auto string_msg = MessageFactory::create_message("std_msgs/String");
  auto int_msg = MessageFactory::create_message("std_msgs/Int32");
  auto twist_msg = MessageFactory::create_message("geometry_msgs/Twist");

  EXPECT_NE(string_msg, nullptr);
  EXPECT_NE(int_msg, nullptr);
  EXPECT_NE(twist_msg, nullptr);

  // Verify types are different (addresses should be different)
  EXPECT_NE(string_msg.get(), int_msg.get());
  EXPECT_NE(int_msg.get(), twist_msg.get());
#elif defined(AXON_ROS2)
  auto string_msg = MessageFactory::create_message("std_msgs/msg/String");
  auto int_msg = MessageFactory::create_message("std_msgs/msg/Int32");
  auto twist_msg = MessageFactory::create_message("geometry_msgs/msg/Twist");

  EXPECT_NE(string_msg, nullptr);
  EXPECT_NE(int_msg, nullptr);
  EXPECT_NE(twist_msg, nullptr);
#endif
}

#endif  // defined(AXON_ROS1) || defined(AXON_ROS2)

// When ROS is not available, provide a placeholder test
#if !defined(AXON_ROS1) && !defined(AXON_ROS2)

TEST(MessageFactoryPlaceholder, ROSRequired) {
  // This test only runs when ROS is not available
  // It serves as a placeholder to indicate the test file was compiled
  GTEST_SKIP() << "MessageFactory tests require ROS (AXON_ROS1 or AXON_ROS2)";
}

#endif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @file test_message_factory_standalone.cpp
 * @brief Unit tests for message_factory.cpp registry pattern
 *
 * Tests the MessageFactory registry pattern without requiring actual ROS message types.
 * This tests the core logic of the factory pattern.
 */

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

// ============================================================================
// Standalone MessageFactory Implementation for Testing
// ============================================================================
// We create a simplified version that matches the interface but doesn't
// require ROS headers. This tests the registry pattern logic.

namespace test {

class TestMessageFactory {
public:
  using MessageCreator = std::function<std::shared_ptr<void>()>;

  struct MessageInfo {
    std::string type_name;
    MessageCreator creator;
  };

  static std::shared_ptr<void> create_message(const std::string& message_type) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end() && it->second.creator) {
      return it->second.creator();
    }
    return nullptr;
  }

  static bool get_message_info(const std::string& message_type, MessageInfo& info) {
    auto& registry = get_registry();
    auto it = registry.find(message_type);
    if (it != registry.end()) {
      info = it->second;
      return true;
    }
    return false;
  }

  static bool is_registered(const std::string& message_type) {
    auto& registry = get_registry();
    return registry.find(message_type) != registry.end();
  }

  // For testing: register a mock message type
  static void register_mock_type(const std::string& message_type) {
    auto& registry = get_registry();
    MessageInfo info;
    info.type_name = message_type;
    info.creator = []() -> std::shared_ptr<void> {
      return std::make_shared<int>(42);  // Dummy value
    };
    registry[message_type] = info;
  }

  // For testing: clear the registry
  static void clear_registry() {
    get_registry().clear();
  }

private:
  static std::unordered_map<std::string, MessageInfo>& get_registry() {
    static std::unordered_map<std::string, MessageInfo> registry;
    return registry;
  }
};

}  // namespace test

using test::TestMessageFactory;

// ============================================================================
// Test Fixture
// ============================================================================

class MessageFactoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    TestMessageFactory::clear_registry();
  }

  void TearDown() override {
    TestMessageFactory::clear_registry();
  }
};

// ============================================================================
// Registry Pattern Tests
// ============================================================================

TEST_F(MessageFactoryTest, GetRegistry_ReturnsConsistentInstance) {
  // Register a type and verify it persists
  TestMessageFactory::register_mock_type("test_msgs/TestMessage");

  EXPECT_TRUE(TestMessageFactory::is_registered("test_msgs/TestMessage"));

  // Should still be registered after second call
  EXPECT_TRUE(TestMessageFactory::is_registered("test_msgs/TestMessage"));
}

TEST_F(MessageFactoryTest, CreateMessage_UnknownType_ReturnsNull) {
  auto msg = TestMessageFactory::create_message("unknown/UnknownType");
  EXPECT_EQ(msg, nullptr);
}

TEST_F(MessageFactoryTest, CreateMessage_RegisteredType_ReturnsInstance) {
  TestMessageFactory::register_mock_type("test_msgs/MockMessage");

  auto msg = TestMessageFactory::create_message("test_msgs/MockMessage");
  EXPECT_NE(msg, nullptr);
}

TEST_F(MessageFactoryTest, IsRegistered_UnknownType_ReturnsFalse) {
  EXPECT_FALSE(TestMessageFactory::is_registered("nonexistent/Type"));
}

TEST_F(MessageFactoryTest, IsRegistered_RegisteredType_ReturnsTrue) {
  TestMessageFactory::register_mock_type("sensor_msgs/Image");
  EXPECT_TRUE(TestMessageFactory::is_registered("sensor_msgs/Image"));
}

TEST_F(MessageFactoryTest, GetMessageInfo_UnknownType_ReturnsFalse) {
  TestMessageFactory::MessageInfo info;
  bool result = TestMessageFactory::get_message_info("unknown/Type", info);
  EXPECT_FALSE(result);
}

TEST_F(MessageFactoryTest, GetMessageInfo_RegisteredType_ReturnsTrue) {
  TestMessageFactory::register_mock_type("geometry_msgs/Twist");

  TestMessageFactory::MessageInfo info;
  bool result = TestMessageFactory::get_message_info("geometry_msgs/Twist", info);

  EXPECT_TRUE(result);
  EXPECT_EQ(info.type_name, "geometry_msgs/Twist");
}

TEST_F(MessageFactoryTest, MultipleTypes_CanBeRegistered) {
  TestMessageFactory::register_mock_type("sensor_msgs/Image");
  TestMessageFactory::register_mock_type("sensor_msgs/Imu");
  TestMessageFactory::register_mock_type("nav_msgs/Odometry");

  EXPECT_TRUE(TestMessageFactory::is_registered("sensor_msgs/Image"));
  EXPECT_TRUE(TestMessageFactory::is_registered("sensor_msgs/Imu"));
  EXPECT_TRUE(TestMessageFactory::is_registered("nav_msgs/Odometry"));
  EXPECT_FALSE(TestMessageFactory::is_registered("sensor_msgs/PointCloud2"));
}

TEST_F(MessageFactoryTest, CreateMessage_MultipleInstances_AreIndependent) {
  TestMessageFactory::register_mock_type("test_msgs/TestMessage");

  auto msg1 = TestMessageFactory::create_message("test_msgs/TestMessage");
  auto msg2 = TestMessageFactory::create_message("test_msgs/TestMessage");

  EXPECT_NE(msg1, nullptr);
  EXPECT_NE(msg2, nullptr);
  EXPECT_NE(msg1.get(), msg2.get());  // Different instances
}

TEST_F(MessageFactoryTest, ClearRegistry_RemovesAllTypes) {
  TestMessageFactory::register_mock_type("type1");
  TestMessageFactory::register_mock_type("type2");

  EXPECT_TRUE(TestMessageFactory::is_registered("type1"));
  EXPECT_TRUE(TestMessageFactory::is_registered("type2"));

  TestMessageFactory::clear_registry();

  EXPECT_FALSE(TestMessageFactory::is_registered("type1"));
  EXPECT_FALSE(TestMessageFactory::is_registered("type2"));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(MessageFactoryTest, EmptyTypeName_NotRegistered) {
  EXPECT_FALSE(TestMessageFactory::is_registered(""));
}

TEST_F(MessageFactoryTest, RegisterEmptyTypeName_CanBeRetrieved) {
  // Edge case: empty string is a valid key
  TestMessageFactory::register_mock_type("");
  EXPECT_TRUE(TestMessageFactory::is_registered(""));
}

TEST_F(MessageFactoryTest, TypeNameWithSpecialChars) {
  // ROS message types can have underscores
  TestMessageFactory::register_mock_type("my_package/My_Message");
  EXPECT_TRUE(TestMessageFactory::is_registered("my_package/My_Message"));
}

TEST_F(MessageFactoryTest, ROS2StyleTypeName) {
  // ROS2 uses package/msg/MessageName format
  TestMessageFactory::register_mock_type("sensor_msgs/msg/Image");
  EXPECT_TRUE(TestMessageFactory::is_registered("sensor_msgs/msg/Image"));

  // But not registered under ROS1 style
  EXPECT_FALSE(TestMessageFactory::is_registered("sensor_msgs/Image"));
}

TEST_F(MessageFactoryTest, GetMessageInfo_PopulatesTypeName) {
  TestMessageFactory::register_mock_type("nav_msgs/msg/Path");

  TestMessageFactory::MessageInfo info;
  TestMessageFactory::get_message_info("nav_msgs/msg/Path", info);

  EXPECT_EQ(info.type_name, "nav_msgs/msg/Path");
  EXPECT_NE(info.creator, nullptr);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

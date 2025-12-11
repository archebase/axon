#ifndef REGISTER_COMMON_MESSAGES_HPP
#define REGISTER_COMMON_MESSAGES_HPP

namespace lance_recorder {
namespace ros1 {

/**
 * Register common ROS message types with the message factory
 * This enables full typed subscription support for these messages
 * 
 * Call this function during initialization to register standard ROS messages
 */
void register_common_message_types();

} // namespace ros1
} // namespace lance_recorder

#endif // REGISTER_COMMON_MESSAGES_HPP


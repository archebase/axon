#ifndef AXON_REGISTER_COMMON_MESSAGES_HPP
#define AXON_REGISTER_COMMON_MESSAGES_HPP

namespace axon {
namespace recorder {

/**
 * Register common ROS message types with the message factory
 * This enables full typed subscription support for these messages
 * 
 * Call this function during initialization to register standard ROS messages
 */
void register_common_message_types();

} // namespace recorder
} // namespace axon

#endif // AXON_REGISTER_COMMON_MESSAGES_HPP

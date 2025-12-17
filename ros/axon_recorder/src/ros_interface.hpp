#ifndef AXON_ROS_INTERFACE_HPP
#define AXON_ROS_INTERFACE_HPP

#include <string>
#include <functional>
#include <memory>

namespace axon {
namespace recorder {

// Forward declarations
class RosNodeHandle;

/**
 * Abstract interface for ROS operations (ROS 1 and ROS 2 compatible)
 * 
 * This interface abstracts away ROS version differences, allowing the same
 * recorder code to work with both ROS 1 and ROS 2. The appropriate implementation
 * is selected at compile time via AXON_ROS1 or AXON_ROS2 macros.
 */
class RosInterface {
public:
    virtual ~RosInterface() = default;
    
    /**
     * Initialize ROS (equivalent to ros::init or rclcpp::init)
     */
    virtual bool init(int argc, char** argv, const std::string& node_name) = 0;
    
    /**
     * Shutdown ROS
     */
    virtual void shutdown() = 0;
    
    /**
     * Check if ROS is running
     */
    virtual bool ok() const = 0;
    
    /**
     * Get the node handle (type-erased)
     */
    virtual void* get_node_handle() = 0;
    
    /**
     * Subscribe to a topic
     * @param topic Topic name
     * @param message_type ROS message type (e.g., "sensor_msgs/Image")
     * @param callback Callback function (void* is the message pointer)
     * @return Subscription handle (can be used to unsubscribe)
     */
    virtual void* subscribe(const std::string& topic,
                            const std::string& message_type,
                            std::function<void(const void*)> callback) = 0;
    
    /**
     * Unsubscribe from a topic
     */
    virtual void unsubscribe(void* subscription_handle) = 0;
    
    /**
     * Advertise a service
     * @param service_name Service name
     * @param service_type Service type (e.g., "std_srvs/Empty")
     * @param callback Service callback (request, response)
     * @return Service handle
     */
    virtual void* advertise_service(const std::string& service_name,
                                   const std::string& service_type,
                                   std::function<bool(const void*, void*)> callback) = 0;
    
    /**
     * Spin once (process callbacks)
     */
    virtual void spin_once() = 0;
    
    /**
     * Spin (blocking)
     */
    virtual void spin() = 0;
    
    /**
     * Get current time in nanoseconds
     */
    virtual int64_t now_nsec() const = 0;
    
    /**
     * Log at different levels
     */
    virtual void log_info(const std::string& message) const = 0;
    virtual void log_warn(const std::string& message) const = 0;
    virtual void log_error(const std::string& message) const = 0;
    virtual void log_debug(const std::string& message) const = 0;
};

/**
 * Factory for creating ROS interface instances
 */
class RosInterfaceFactory {
public:
    /**
     * Create ROS interface instance
     * Returns the appropriate implementation based on compile-time flags
     */
    static std::unique_ptr<RosInterface> create();
};

} // namespace recorder
} // namespace axon

#endif // AXON_ROS_INTERFACE_HPP

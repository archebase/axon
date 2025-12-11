#include "../common/ros_interface.hpp"
#include "message_factory.hpp"
#include "register_common_messages.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/message.h>
#include <ros/service_traits.h>
#include <ros/connection.h>
#include <ros/transport_hints.h>
#include <ros/subscribe_options.h>
#include <ros/topic_manager.h>
#include <memory>
#include <functional>
#include <map>
#include <string>

namespace lance_recorder {
namespace common {

class Ros1Interface : public RosInterface {
public:
    Ros1Interface() : node_handle_(nullptr), initialized_(false) {}
    
    ~Ros1Interface() override {
        shutdown();
        // Cleanup subscriptions
        for (auto& pair : subscriptions_) {
            delete static_cast<ros::Subscriber*>(pair.first);
        }
        subscriptions_.clear();
        // Services are automatically cleaned up by ROS
        services_.clear();
    }
    
    bool init(int argc, char** argv, const std::string& node_name) override {
        if (initialized_) {
            return true;
        }
        
        ros::init(argc, argv, node_name, ros::init_options::AnonymousName);
        if (!ros::ok()) {
            return false;
        }
        
        node_handle_ = std::make_unique<ros::NodeHandle>();
        
        // Register common message types for full typed subscription support
        ros1::register_common_message_types();
        
        initialized_ = true;
        return true;
    }
    
    void shutdown() override {
        if (initialized_) {
            // Unsubscribe all before shutdown
            for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
                auto* sub = static_cast<ros::Subscriber*>(it->first);
                subscriptions_.erase(it++);
                delete sub;
            }
            
            ros::shutdown();
            node_handle_.reset();
            initialized_ = false;
        }
    }
    
    bool ok() const override {
        return ros::ok() && initialized_;
    }
    
    void* get_node_handle() override {
        return node_handle_.get();
    }
    
    void* subscribe(const std::string& topic,
                   const std::string& message_type,
                   std::function<void(const void*)> callback) override {
        if (!node_handle_) {
            return nullptr;
        }
        
        ros::Subscriber* sub = nullptr;
        
        try {
            // Use ROS1's SubscribeOptions for full control over subscription
            ros::SubscribeOptions opts;
            opts.topic = topic;
            opts.queue_size = 10;
            opts.transport_hints = ros::TransportHints();
            
            // Set message type information
            if (ros1::MessageFactory::is_registered(message_type)) {
                // Use registered message type info
                ros1::MessageFactory::MessageInfo info;
                if (ros1::MessageFactory::get_message_info(message_type, info)) {
                    opts.md5sum = info.md5sum;
                    opts.datatype = info.datatype;
                }
            } else {
                // Use provided message type (ROS will resolve MD5)
                opts.datatype = message_type;
            }
            
            // Create callback that handles message deserialization
            // Use ROS1's standard message callback signature
            auto msg_callback = [callback, message_type](const ros::MessageConstPtr& msg) {
                if (!msg) {
                    return;
                }
                
                // If message type is registered in factory, deserialize into typed message
                if (ros1::MessageFactory::is_registered(message_type)) {
                    auto typed_msg = ros1::MessageFactory::create_message(message_type);
                    if (typed_msg) {
                        // Deserialize raw message data into typed message
                        const uint8_t* data = msg->raw();
                        uint32_t size = msg->size();
                        if (data && size > 0) {
                            try {
                                ros1::MessageFactory::deserialize_message(message_type, data, size, *typed_msg);
                                callback(static_cast<const void*>(typed_msg.get()));
                                return;
                            } catch (const std::exception& e) {
                                ROS_WARN("Failed to deserialize message type %s: %s", 
                                        message_type.c_str(), e.what());
                            }
                        }
                    }
                }
                
                // Fallback: use raw message pointer
                callback(static_cast<const void*>(msg.get()));
            };
            
            // Create subscription using standard ROS1 API
            // The callback handles message type deserialization via MessageFactory
            // ROS1 automatically resolves message type from the topic connection
            sub = new ros::Subscriber(
                node_handle_->subscribe(opts.topic, opts.queue_size, msg_callback)
            );
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to subscribe to topic %s (type: %s): %s", 
                     topic.c_str(), message_type.c_str(), e.what());
            if (sub) {
                delete sub;
                sub = nullptr;
            }
        }
        
        if (sub) {
            subscriptions_[sub] = std::make_pair(topic, message_type);
            ROS_INFO("Successfully subscribed to topic %s (type: %s)", 
                    topic.c_str(), message_type.c_str());
        }
        
        return sub;
    }
    
    void unsubscribe(void* subscription_handle) override {
        if (!subscription_handle) {
            return;
        }
        
        auto it = subscriptions_.find(subscription_handle);
        if (it != subscriptions_.end()) {
            auto* sub = static_cast<ros::Subscriber*>(subscription_handle);
            subscriptions_.erase(it);
            delete sub;
        }
    }
    
    void* advertise_service(const std::string& service_name,
                           const std::string& service_type,
                           std::function<bool(const void*, void*)> callback) override {
        if (!node_handle_) {
            return nullptr;
        }
        
        // Store service callback
        auto* service_wrapper = new ServiceWrapper();
        service_wrapper->callback = callback;
        service_wrapper->service_name = service_name;
        service_wrapper->service_type = service_type;
        
        // Advertise service using ROS1 service system
        try {
            // Create service callback
            auto service_callback = [service_wrapper](ros::ServiceRequest& req, 
                                                    ros::ServiceResponse& res) -> bool {
                if (service_wrapper->callback) {
                    return service_wrapper->callback(
                        static_cast<const void*>(&req),
                        static_cast<void*>(&res)
                    );
                }
                return false;
            };
            
            // Use standard advertiseService - ROS will handle type resolution
            service_wrapper->server = node_handle_->advertiseService(
                service_name,
                service_callback
            );
            
            if (!service_wrapper->server) {
                throw std::runtime_error("Failed to create service server");
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to advertise service %s (type: %s): %s", 
                     service_name.c_str(), service_type.c_str(), e.what());
            delete service_wrapper;
            return nullptr;
        }
        
        services_[service_wrapper] = std::make_pair(service_name, service_type);
        ROS_INFO("Successfully advertised service %s (type: %s)", 
                service_name.c_str(), service_type.c_str());
        return service_wrapper;
    }
    
    void spin_once() override {
        ros::spinOnce();
    }
    
    void spin() override {
        ros::spin();
    }
    
    int64_t now_nsec() const override {
        auto now = ros::Time::now();
        return static_cast<int64_t>(now.sec) * 1000000000LL + static_cast<int64_t>(now.nsec);
    }
    
    void log_info(const std::string& message) const override {
        ROS_INFO("%s", message.c_str());
    }
    
    void log_warn(const std::string& message) const override {
        ROS_WARN("%s", message.c_str());
    }
    
    void log_error(const std::string& message) const override {
        ROS_ERROR("%s", message.c_str());
    }
    
    void log_debug(const std::string& message) const override {
        ROS_DEBUG("%s", message.c_str());
    }
    
private:
    struct ServiceWrapper {
        ros::ServiceServer server;
        std::function<bool(const void*, void*)> callback;
        std::string service_name;
        std::string service_type;
    };
    
    std::unique_ptr<ros::NodeHandle> node_handle_;
    bool initialized_;
    std::map<void*, std::pair<std::string, std::string>> subscriptions_;
    std::map<void*, std::pair<std::string, std::string>> services_;
};

RosVersion RosInterfaceFactory::detect_ros_version() {
    // Check for ROS 1
    const char* ros_distro = std::getenv("ROS_DISTRO");
    if (ros_distro) {
        // ROS 1 distros: noetic, melodic, kinetic, etc.
        std::string distro(ros_distro);
        if (distro == "noetic" || distro == "melodic" || distro == "kinetic") {
            return RosVersion::ROS1;
        }
    }
    
    // Check for ROS 2
    const char* ros_version = std::getenv("ROS_VERSION");
    if (ros_version) {
        std::string version(ros_version);
        if (version == "2") {
            return RosVersion::ROS2;
        }
    }
    
    // Default to ROS 1 for backward compatibility
    return RosVersion::ROS1;
}

std::unique_ptr<RosInterface> RosInterfaceFactory::create(RosVersion version) {
    if (version == RosVersion::ROS1) {
        return std::make_unique<Ros1Interface>();
    } else {
        // ROS 2 implementation will be in ros2_interface.cpp
        return nullptr;
    }
}

} // namespace common
} // namespace lance_recorder

#include "../common/ros_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>
#include <memory>
#include <functional>
#include <map>
#include <string>

namespace lance_recorder {
namespace common {

class Ros2Interface : public RosInterface {
public:
    Ros2Interface() : node_(nullptr), initialized_(false) {}
    
    ~Ros2Interface() override {
        shutdown();
    }
    
    bool init(int argc, char** argv, const std::string& node_name) override {
        if (initialized_) {
            return true;
        }
        
        rclcpp::init(argc, argv);
        if (!rclcpp::ok()) {
            return false;
        }
        
        node_ = std::make_shared<rclcpp::Node>(node_name);
        initialized_ = true;
        return true;
    }
    
    void shutdown() override {
        if (initialized_) {
            // Clear subscriptions and services (they're shared_ptrs, so auto-cleanup)
            subscriptions_.clear();
            services_.clear();
            node_.reset();
            rclcpp::shutdown();
            initialized_ = false;
        }
    }
    
    bool ok() const override {
        return rclcpp::ok() && initialized_;
    }
    
    void* get_node_handle() override {
        return node_.get();
    }
    
    void* subscribe(const std::string& topic,
                   const std::string& message_type,
                   std::function<void(const void*)> callback) override {
        if (!node_) {
            return nullptr;
        }
        
        // Create a generic subscription
        // Note: Full implementation would use message type traits and factory
        // For now, we create a type-erased subscription wrapper
        try {
            auto sub_wrapper = std::make_shared<SubscriptionWrapper>();
            sub_wrapper->callback = callback;
            sub_wrapper->topic = topic;
            sub_wrapper->message_type = message_type;
            
            // Create subscription using rclcpp::create_generic_subscription
            // Note: This requires ROS 2 Foxy or later. For earlier versions,
            // you would need to use typed subscriptions or message_factory
            try {
                sub_wrapper->subscription = rclcpp::create_generic_subscription(
                    node_->get_node_topics_interface(),
                    topic,
                    message_type,
                    rclcpp::QoS(10),
                    [sub_wrapper](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                        if (sub_wrapper->callback && msg) {
                            // Pass serialized message pointer
                            sub_wrapper->callback(static_cast<const void*>(msg.get()));
                        }
                    }
                );
                
                if (sub_wrapper->subscription) {
                    // Store wrapper
                    auto* wrapper_ptr = new std::shared_ptr<SubscriptionWrapper>(sub_wrapper);
                    subscriptions_[wrapper_ptr] = std::make_pair(topic, message_type);
                    return wrapper_ptr;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), 
                            "create_generic_subscription failed (may require ROS 2 Foxy+): %s", 
                            e.what());
                // Fallback: store wrapper anyway for future use
                auto* wrapper_ptr = new std::shared_ptr<SubscriptionWrapper>(sub_wrapper);
                subscriptions_[wrapper_ptr] = std::make_pair(topic, message_type);
                return wrapper_ptr;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to subscribe to topic %s: %s", 
                        topic.c_str(), e.what());
        }
        
        return nullptr;
    }
    
    void unsubscribe(void* subscription_handle) override {
        if (!subscription_handle) {
            return;
        }
        
        auto it = subscriptions_.find(subscription_handle);
        if (it != subscriptions_.end()) {
            auto* wrapper_ptr = static_cast<std::shared_ptr<SubscriptionWrapper>*>(subscription_handle);
            subscriptions_.erase(it);
            delete wrapper_ptr;
        }
    }
    
    void* advertise_service(const std::string& service_name,
                           const std::string& service_type,
                           std::function<bool(const void*, void*)> callback) override {
        if (!node_) {
            return nullptr;
        }
        
        try {
            auto service_wrapper = std::make_shared<ServiceWrapper>();
            service_wrapper->callback = callback;
            service_wrapper->service_name = service_name;
            service_wrapper->service_type = service_type;
            
            // Generic services are not available in all ROS 2 versions
            // For now, store the wrapper for typed service registration
            // The actual service needs to be created with concrete types
            // through the ROS 2 service interfaces (see edge_lance_recorder_node.cpp)
            RCLCPP_WARN(node_->get_logger(),
                       "Generic service '%s' registered but requires typed implementation",
                       service_name.c_str());
            
            auto* wrapper_ptr = new std::shared_ptr<ServiceWrapper>(service_wrapper);
            services_[wrapper_ptr] = std::make_pair(service_name, service_type);
            return wrapper_ptr;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to advertise service %s: %s",
                        service_name.c_str(), e.what());
        }
        
        return nullptr;
    }
    
    void spin_once() override {
        rclcpp::spin_some(node_);
    }
    
    void spin() override {
        rclcpp::spin(node_);
    }
    
    int64_t now_nsec() const override {
        if (!node_) {
            return 0;
        }
        auto now = node_->now();
        return now.nanoseconds();
    }
    
    void log_info(const std::string& message) const override {
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
        }
    }
    
    void log_warn(const std::string& message) const override {
        if (node_) {
            RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
        }
    }
    
    void log_error(const std::string& message) const override {
        if (node_) {
            RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
        }
    }
    
    void log_debug(const std::string& message) const override {
        if (node_) {
            RCLCPP_DEBUG(node_->get_logger(), "%s", message.c_str());
        }
    }
    
private:
    struct SubscriptionWrapper {
        std::function<void(const void*)> callback;
        rclcpp::GenericSubscription::SharedPtr subscription;
        std::string topic;
        std::string message_type;
    };
    
    struct ServiceWrapper {
        std::function<bool(const void*, void*)> callback;
        // GenericService may not be available in all ROS2 versions
        // Using void* for now as a placeholder
        std::shared_ptr<void> service;
        std::string service_name;
        std::string service_type;
    };
    
    std::shared_ptr<rclcpp::Node> node_;
    bool initialized_;
    std::map<void*, std::pair<std::string, std::string>> subscriptions_;
    std::map<void*, std::pair<std::string, std::string>> services_;
};

// Update factory to support ROS 2
std::unique_ptr<RosInterface> RosInterfaceFactory::create(RosVersion version) {
    if (version == RosVersion::ROS1) {
        // ROS 1 implementation is in ros1_interface.cpp
        return nullptr; // Will be linked separately
    } else if (version == RosVersion::ROS2) {
        return std::make_unique<Ros2Interface>();
    }
    return nullptr;
}

} // namespace common
} // namespace lance_recorder

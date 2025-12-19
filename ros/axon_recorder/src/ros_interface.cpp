#include "ros_interface.hpp"
#include "message_factory.hpp"
#include "register_common_messages.hpp"

#if defined(AXON_ROS1)
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/message.h>
#include <ros/service_traits.h>
#include <ros/connection.h>
#include <ros/transport_hints.h>
#include <ros/subscribe_options.h>
#include <ros/topic_manager.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rmw/rmw.h>
#endif

#include <memory>
#include <functional>
#include <map>
#include <string>
#include <cstdlib>

namespace axon {
namespace recorder {

#if defined(AXON_ROS1)
// ============================================================================
// ROS 1 Implementation
// ============================================================================

class RosInterfaceImpl : public RosInterface {
public:
    RosInterfaceImpl() : node_handle_(nullptr), initialized_(false) {}
    
    ~RosInterfaceImpl() override {
        shutdown();
        // Cleanup subscriptions
        for (auto& pair : subscriptions_) {
            delete static_cast<ros::Subscriber*>(pair.first);
        }
        subscriptions_.clear();
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
        register_common_message_types();
        
        initialized_ = true;
        return true;
    }
    
    void shutdown() override {
        if (initialized_) {
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
            ros::SubscribeOptions opts;
            opts.topic = topic;
            opts.queue_size = 10;
            opts.transport_hints = ros::TransportHints();
            
            if (MessageFactory::is_registered(message_type)) {
                MessageFactory::MessageInfo info;
                if (MessageFactory::get_message_info(message_type, info)) {
                    opts.md5sum = info.md5sum;
                    opts.datatype = info.datatype;
                }
            } else {
                opts.datatype = message_type;
            }
            
            auto msg_callback = [callback, message_type](const ros::MessageConstPtr& msg) {
                if (!msg) {
                    return;
                }
                
                if (MessageFactory::is_registered(message_type)) {
                    auto typed_msg = MessageFactory::create_message(message_type);
                    if (typed_msg) {
                        const uint8_t* data = msg->raw();
                        uint32_t size = msg->size();
                        if (data && size > 0) {
                            try {
                                MessageFactory::deserialize_message(message_type, data, size, *typed_msg);
                                callback(static_cast<const void*>(typed_msg.get()));
                                return;
                            } catch (const std::exception& e) {
                                ROS_WARN("Failed to deserialize message type %s: %s", 
                                        message_type.c_str(), e.what());
                            }
                        }
                    }
                }
                
                callback(static_cast<const void*>(msg.get()));
            };
            
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
        
        auto* service_wrapper = new ServiceWrapper();
        service_wrapper->callback = callback;
        service_wrapper->service_name = service_name;
        service_wrapper->service_type = service_type;
        
        try {
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

#elif defined(AXON_ROS2)
// ============================================================================
// ROS 2 Implementation
// ============================================================================

class RosInterfaceImpl : public RosInterface {
public:
    RosInterfaceImpl() : node_(nullptr), initialized_(false), num_threads_(8) {}
    
    ~RosInterfaceImpl() override {
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
        
        // Create node with options that allow reentrant callbacks
        rclcpp::NodeOptions options;
        options.allow_undeclared_parameters(true);
        options.automatically_declare_parameters_from_overrides(true);
        
        node_ = std::make_shared<rclcpp::Node>(node_name, options);
        
        // Create callback group for parallel processing
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        
        // Create multi-threaded executor for parallel callback processing
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), num_threads_);
        executor_->add_node(node_);
        
        initialized_ = true;
        return true;
    }
    
    void shutdown() override {
        if (initialized_) {
            // Cancel executor before cleaning up
            if (executor_) {
                executor_->cancel();
                executor_.reset();
            }
            subscriptions_.clear();
            services_.clear();
            callback_group_.reset();
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
        
        try {
            auto sub_wrapper = std::make_shared<SubscriptionWrapper>();
            sub_wrapper->callback = callback;
            sub_wrapper->topic = topic;
            sub_wrapper->message_type = message_type;
            
            try {
                // Use reliable QoS with large queue for high-throughput recording
                auto qos = rclcpp::QoS(1000)
                    .reliable()
                    .keep_last(1000)
                    .durability_volatile();
                
                // Create subscription options with reentrant callback group
                rclcpp::SubscriptionOptions sub_options;
                sub_options.callback_group = callback_group_;
                    
                sub_wrapper->subscription = rclcpp::create_generic_subscription(
                    node_->get_node_topics_interface(),
                    topic,
                    message_type,
                    qos,
                    [sub_wrapper](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                        if (sub_wrapper->callback && msg) {
                            sub_wrapper->callback(static_cast<const void*>(msg.get()));
                        }
                    },
                    sub_options
                );
                
                if (sub_wrapper->subscription) {
                    auto* wrapper_ptr = new std::shared_ptr<SubscriptionWrapper>(sub_wrapper);
                    subscriptions_[wrapper_ptr] = std::make_pair(topic, message_type);
                    return wrapper_ptr;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), 
                            "create_generic_subscription failed (may require ROS 2 Foxy+): %s", 
                            e.what());
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
        if (executor_) {
            executor_->spin_some();
        }
    }
    
    void spin() override {
        if (executor_) {
            executor_->spin();
        }
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
        std::shared_ptr<void> service;
        std::string service_name;
        std::string service_type;
    };
    
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    bool initialized_;
    size_t num_threads_;
    std::map<void*, std::pair<std::string, std::string>> subscriptions_;
    std::map<void*, std::pair<std::string, std::string>> services_;
};

#else
#error "Either AXON_ROS1 or AXON_ROS2 must be defined"
#endif

// ============================================================================
// Factory Implementation
// ============================================================================

std::unique_ptr<RosInterface> RosInterfaceFactory::create() {
    return std::make_unique<RosInterfaceImpl>();
}

} // namespace recorder
} // namespace axon

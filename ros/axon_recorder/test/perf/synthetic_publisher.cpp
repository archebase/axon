//! Synthetic Data Publisher for Performance Testing
//!
//! Publishes synthetic sensor data for performance testing:
//! - IMU at 1000Hz (sensor_msgs/Imu)
//! - Camera frames at 30Hz (sensor_msgs/Image)
//! - Depth frames at 30Hz (sensor_msgs/Image)
//!
//! Usage:
//!   ROS1: rosrun axon_recorder synthetic_publisher [options]
//!   ROS2: ros2 run axon_recorder synthetic_publisher [options]
//!
//! Options:
//!   --imu-rate <hz>       IMU publish rate (default: 1000)
//!   --camera-rate <hz>    Camera publish rate (default: 30)
//!   --num-cameras <n>     Number of cameras (default: 3)
//!   --width <px>          Image width (default: 1920)
//!   --height <px>         Image height (default: 1080)
//!   --duration <sec>      Test duration in seconds (default: 10)
//!   --compress            Enable JPEG compression for RGB

#include <cstdlib>
#include <cstring>
#include <chrono>
#include <random>
#include <thread>
#include <atomic>
#include <iostream>
#include <vector>

#if defined(AXON_ROS1)
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#elif defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace axon {
namespace test {

// Configuration structure
struct PublisherConfig {
    int imu_rate = 1000;       // Hz
    int camera_rate = 30;      // Hz
    int num_cameras = 3;
    int width = 1920;
    int height = 1080;
    int duration_sec = 10;
    bool compress = false;
    
    void parse_args(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            if (strcmp(argv[i], "--imu-rate") == 0 && i + 1 < argc) {
                imu_rate = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--camera-rate") == 0 && i + 1 < argc) {
                camera_rate = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--num-cameras") == 0 && i + 1 < argc) {
                num_cameras = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
                width = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
                height = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
                duration_sec = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--compress") == 0) {
                compress = true;
            } else if (strcmp(argv[i], "--help") == 0) {
                print_usage();
                std::exit(0);
            }
        }
    }
    
    void print_usage() {
        std::cout << "Synthetic Publisher for Performance Testing\n"
                  << "Usage: synthetic_publisher [options]\n"
                  << "Options:\n"
                  << "  --imu-rate <hz>       IMU publish rate (default: 1000)\n"
                  << "  --camera-rate <hz>    Camera publish rate (default: 30)\n"
                  << "  --num-cameras <n>     Number of cameras (default: 3)\n"
                  << "  --width <px>          Image width (default: 1920)\n"
                  << "  --height <px>         Image height (default: 1080)\n"
                  << "  --duration <sec>      Test duration (default: 10)\n"
                  << "  --compress            Enable JPEG compression\n"
                  << "  --help                Show this help\n";
    }
    
    void print_config() {
        std::cout << "=== Synthetic Publisher Configuration ===\n"
                  << "IMU Rate: " << imu_rate << " Hz\n"
                  << "Camera Rate: " << camera_rate << " Hz\n"
                  << "Number of Cameras: " << num_cameras << "\n"
                  << "Resolution: " << width << "x" << height << "\n"
                  << "Duration: " << duration_sec << " seconds\n"
                  << "Compression: " << (compress ? "enabled" : "disabled") << "\n"
                  << "========================================\n";
    }
};

// Statistics tracking
struct PublishStats {
    std::atomic<uint64_t> imu_published{0};
    std::atomic<uint64_t> rgb_published{0};
    std::atomic<uint64_t> depth_published{0};
    std::atomic<uint64_t> total_bytes{0};
    std::chrono::steady_clock::time_point start_time;
    
    void print() {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        
        std::cout << "\n=== Publishing Statistics ===\n"
                  << "Elapsed: " << elapsed << " seconds\n"
                  << "IMU Messages: " << imu_published.load() << "\n"
                  << "RGB Frames: " << rgb_published.load() << "\n"
                  << "Depth Frames: " << depth_published.load() << "\n"
                  << "Total Bytes: " << (total_bytes.load() / 1024 / 1024) << " MB\n"
                  << "Throughput: " << (total_bytes.load() / 1024 / 1024 / elapsed) << " MB/s\n"
                  << "============================\n";
    }
};

// Generate synthetic IMU data
#if defined(AXON_ROS1)
sensor_msgs::Imu generate_imu_message(std::mt19937& rng) {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "imu_link";
    
    std::normal_distribution<double> noise(0.0, 0.01);
    
    // Linear acceleration (with gravity + noise)
    msg.linear_acceleration.x = noise(rng);
    msg.linear_acceleration.y = noise(rng);
    msg.linear_acceleration.z = 9.81 + noise(rng);
    
    // Angular velocity
    msg.angular_velocity.x = noise(rng) * 0.1;
    msg.angular_velocity.y = noise(rng) * 0.1;
    msg.angular_velocity.z = noise(rng) * 0.1;
    
    // Orientation quaternion
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    
    return msg;
}
#elif defined(AXON_ROS2)
sensor_msgs::msg::Imu generate_imu_message(rclcpp::Node& node, std::mt19937& rng) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node.now();
    msg.header.frame_id = "imu_link";
    
    std::normal_distribution<double> noise(0.0, 0.01);
    
    msg.linear_acceleration.x = noise(rng);
    msg.linear_acceleration.y = noise(rng);
    msg.linear_acceleration.z = 9.81 + noise(rng);
    
    msg.angular_velocity.x = noise(rng) * 0.1;
    msg.angular_velocity.y = noise(rng) * 0.1;
    msg.angular_velocity.z = noise(rng) * 0.1;
    
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    
    return msg;
}
#endif

// Generate synthetic image data
#if defined(AXON_ROS1)
sensor_msgs::Image generate_rgb_image(int width, int height, int camera_id, std::mt19937& rng) {
    sensor_msgs::Image msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_" + std::to_string(camera_id) + "_rgb_frame";
    
    msg.width = width;
    msg.height = height;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width * 3;
    
    // Generate patterned data (simulates real camera gradient patterns)
    msg.data.resize(width * height * 3);
    std::uniform_int_distribution<int> noise(-5, 5);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            int base = ((x + y) % 256);
            msg.data[idx] = std::clamp(base + noise(rng), 0, 255);
            msg.data[idx + 1] = std::clamp((base + 85) % 256 + noise(rng), 0, 255);
            msg.data[idx + 2] = std::clamp((base + 170) % 256 + noise(rng), 0, 255);
        }
    }
    
    return msg;
}

sensor_msgs::Image generate_depth_image(int width, int height, int camera_id, std::mt19937& rng) {
    sensor_msgs::Image msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_" + std::to_string(camera_id) + "_depth_frame";
    
    msg.width = width;
    msg.height = height;
    msg.encoding = "16UC1";  // 16-bit depth
    msg.is_bigendian = false;
    msg.step = width * 2;
    
    // Generate depth data (simulates depth camera pattern)
    msg.data.resize(width * height * 2);
    std::normal_distribution<double> depth_noise(0.0, 10.0);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 2;
            // Simulate depth from 500mm to 10000mm with some variation
            uint16_t depth = static_cast<uint16_t>(
                std::clamp(2000.0 + 500.0 * std::sin(x * 0.01) + depth_noise(rng), 500.0, 10000.0));
            msg.data[idx] = depth & 0xFF;
            msg.data[idx + 1] = (depth >> 8) & 0xFF;
        }
    }
    
    return msg;
}
#elif defined(AXON_ROS2)
sensor_msgs::msg::Image generate_rgb_image(rclcpp::Node& node, int width, int height, int camera_id, std::mt19937& rng) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node.now();
    msg.header.frame_id = "camera_" + std::to_string(camera_id) + "_rgb_frame";
    
    msg.width = width;
    msg.height = height;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = width * 3;
    
    msg.data.resize(width * height * 3);
    std::uniform_int_distribution<int> noise(-5, 5);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 3;
            int base = ((x + y) % 256);
            msg.data[idx] = std::clamp(base + noise(rng), 0, 255);
            msg.data[idx + 1] = std::clamp((base + 85) % 256 + noise(rng), 0, 255);
            msg.data[idx + 2] = std::clamp((base + 170) % 256 + noise(rng), 0, 255);
        }
    }
    
    return msg;
}

sensor_msgs::msg::Image generate_depth_image(rclcpp::Node& node, int width, int height, int camera_id, std::mt19937& rng) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node.now();
    msg.header.frame_id = "camera_" + std::to_string(camera_id) + "_depth_frame";
    
    msg.width = width;
    msg.height = height;
    msg.encoding = "16UC1";
    msg.is_bigendian = false;
    msg.step = width * 2;
    
    msg.data.resize(width * height * 2);
    std::normal_distribution<double> depth_noise(0.0, 10.0);
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = (y * width + x) * 2;
            uint16_t depth = static_cast<uint16_t>(
                std::clamp(2000.0 + 500.0 * std::sin(x * 0.01) + depth_noise(rng), 500.0, 10000.0));
            msg.data[idx] = depth & 0xFF;
            msg.data[idx + 1] = (depth >> 8) & 0xFF;
        }
    }
    
    return msg;
}
#endif

} // namespace test
} // namespace axon

#if defined(AXON_ROS1)
// =============================================================================
// ROS 1 Implementation
// =============================================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "synthetic_publisher");
    ros::NodeHandle nh;
    
    axon::test::PublisherConfig config;
    config.parse_args(argc, argv);
    config.print_config();
    
    axon::test::PublishStats stats;
    stats.start_time = std::chrono::steady_clock::now();
    
    // Create publishers
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
    
    std::vector<ros::Publisher> rgb_pubs, depth_pubs;
    for (int i = 0; i < config.num_cameras; ++i) {
        rgb_pubs.push_back(
            nh.advertise<sensor_msgs::Image>("/camera" + std::to_string(i) + "/rgb", 10));
        depth_pubs.push_back(
            nh.advertise<sensor_msgs::Image>("/camera" + std::to_string(i) + "/depth", 10));
    }
    
    std::mt19937 rng(42);
    std::atomic<bool> running{true};
    
    // IMU publisher thread
    std::thread imu_thread([&]() {
        ros::Rate rate(config.imu_rate);
        while (running && ros::ok()) {
            auto msg = axon::test::generate_imu_message(rng);
            imu_pub.publish(msg);
            stats.imu_published++;
            stats.total_bytes += sizeof(sensor_msgs::Imu);
            rate.sleep();
        }
    });
    
    // Camera publisher thread
    std::thread camera_thread([&]() {
        ros::Rate rate(config.camera_rate);
        while (running && ros::ok()) {
            for (int i = 0; i < config.num_cameras; ++i) {
                auto rgb_msg = axon::test::generate_rgb_image(
                    config.width, config.height, i, rng);
                rgb_pubs[i].publish(rgb_msg);
                stats.rgb_published++;
                stats.total_bytes += rgb_msg.data.size();
                
                auto depth_msg = axon::test::generate_depth_image(
                    config.width, config.height, i, rng);
                depth_pubs[i].publish(depth_msg);
                stats.depth_published++;
                stats.total_bytes += depth_msg.data.size();
            }
            rate.sleep();
        }
    });
    
    // Status printer thread
    std::thread status_thread([&]() {
        while (running && ros::ok()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            stats.print();
        }
    });
    
    // Run for specified duration
    std::this_thread::sleep_for(std::chrono::seconds(config.duration_sec));
    running = false;
    
    imu_thread.join();
    camera_thread.join();
    status_thread.join();
    
    std::cout << "\n=== Final Statistics ===\n";
    stats.print();
    
    return 0;
}

#elif defined(AXON_ROS2)
// =============================================================================
// ROS 2 Implementation
// =============================================================================

class SyntheticPublisher : public rclcpp::Node {
public:
    SyntheticPublisher(const axon::test::PublisherConfig& config)
        : Node("synthetic_publisher")
        , config_(config)
        , rng_(42)
        , running_(true)
    {
        stats_.start_time = std::chrono::steady_clock::now();
        
        // Create publishers
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 100);
        
        for (int i = 0; i < config_.num_cameras; ++i) {
            rgb_pubs_.push_back(
                create_publisher<sensor_msgs::msg::Image>("/camera" + std::to_string(i) + "/rgb", 10));
            depth_pubs_.push_back(
                create_publisher<sensor_msgs::msg::Image>("/camera" + std::to_string(i) + "/depth", 10));
        }
        
        // Create timers
        auto imu_period = std::chrono::microseconds(1000000 / config_.imu_rate);
        imu_timer_ = create_wall_timer(imu_period, 
            std::bind(&SyntheticPublisher::imu_callback, this));
        
        auto camera_period = std::chrono::microseconds(1000000 / config_.camera_rate);
        camera_timer_ = create_wall_timer(camera_period,
            std::bind(&SyntheticPublisher::camera_callback, this));
        
        // Status timer
        status_timer_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&SyntheticPublisher::status_callback, this));
        
        // Duration timer
        duration_timer_ = create_wall_timer(
            std::chrono::seconds(config_.duration_sec),
            [this]() {
                RCLCPP_INFO(get_logger(), "Test duration complete, shutting down...");
                stats_.print();
                rclcpp::shutdown();
            });
    }
    
private:
    void imu_callback() {
        auto msg = axon::test::generate_imu_message(*this, rng_);
        imu_pub_->publish(msg);
        stats_.imu_published++;
        stats_.total_bytes += sizeof(sensor_msgs::msg::Imu);
    }
    
    void camera_callback() {
        for (int i = 0; i < config_.num_cameras; ++i) {
            auto rgb_msg = axon::test::generate_rgb_image(
                *this, config_.width, config_.height, i, rng_);
            rgb_pubs_[i]->publish(rgb_msg);
            stats_.rgb_published++;
            stats_.total_bytes += rgb_msg.data.size();
            
            auto depth_msg = axon::test::generate_depth_image(
                *this, config_.width, config_.height, i, rng_);
            depth_pubs_[i]->publish(depth_msg);
            stats_.depth_published++;
            stats_.total_bytes += depth_msg.data.size();
        }
    }
    
    void status_callback() {
        stats_.print();
    }
    
    axon::test::PublisherConfig config_;
    axon::test::PublishStats stats_;
    std::mt19937 rng_;
    std::atomic<bool> running_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rgb_pubs_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_pubs_;
    
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr duration_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    axon::test::PublisherConfig config;
    config.parse_args(argc, argv);
    config.print_config();
    
    auto node = std::make_shared<SyntheticPublisher>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

#else
// =============================================================================
// Standalone test (no ROS)
// =============================================================================

int main(int argc, char** argv) {
    std::cout << "This program requires ROS. Compile with -DAXON_ROS1 or -DAXON_ROS2\n";
    return 1;
}

#endif

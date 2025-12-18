//! Performance Test Node
//!
//! This node monitors the axon_recorder performance during recording:
//! - CPU usage (target: <10%)
//! - Memory usage
//! - Message drop rate (target: 0%)
//! - Write latency percentiles
//!
//! Usage:
//!   1. Start the synthetic_publisher
//!   2. Start axon_recorder with recording enabled
//!   3. Run this performance test node
//!   4. Results are printed and optionally saved to JSON

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <thread>
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

// Platform-specific includes for system metrics
#ifdef __linux__
#include <sys/resource.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#elif __APPLE__
#include <mach/mach.h>
#include <sys/resource.h>
#include <sys/sysctl.h>
#endif

namespace axon {
namespace test {

// =============================================================================
// System Metrics Collection
// =============================================================================

struct SystemMetrics {
    double cpu_percent = 0.0;
    size_t memory_bytes = 0;
    size_t rss_bytes = 0;  // Resident Set Size
    
    std::string to_string() const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2)
           << "CPU: " << cpu_percent << "%, "
           << "Memory: " << (memory_bytes / 1024 / 1024) << " MB, "
           << "RSS: " << (rss_bytes / 1024 / 1024) << " MB";
        return ss.str();
    }
};

class SystemMonitor {
public:
    SystemMonitor() {
        // Initialize baseline
        update();
    }
    
    void update() {
        current_ = collect_metrics();
        samples_.push_back(current_);
        
        // Keep only last N samples
        if (samples_.size() > max_samples_) {
            samples_.erase(samples_.begin());
        }
    }
    
    SystemMetrics current() const { return current_; }
    
    SystemMetrics average() const {
        if (samples_.empty()) return {};
        
        SystemMetrics avg;
        for (const auto& s : samples_) {
            avg.cpu_percent += s.cpu_percent;
            avg.memory_bytes += s.memory_bytes;
            avg.rss_bytes += s.rss_bytes;
        }
        avg.cpu_percent /= samples_.size();
        avg.memory_bytes /= samples_.size();
        avg.rss_bytes /= samples_.size();
        return avg;
    }
    
    double peak_cpu() const {
        double peak = 0;
        for (const auto& s : samples_) {
            peak = std::max(peak, s.cpu_percent);
        }
        return peak;
    }
    
    size_t peak_memory() const {
        size_t peak = 0;
        for (const auto& s : samples_) {
            peak = std::max(peak, s.memory_bytes);
        }
        return peak;
    }

private:
    SystemMetrics collect_metrics() {
        SystemMetrics metrics;
        
#ifdef __linux__
        // CPU usage from /proc/stat
        std::ifstream stat_file("/proc/stat");
        std::string line;
        std::getline(stat_file, line);
        
        long user, nice, system, idle, iowait, irq, softirq;
        sscanf(line.c_str(), "cpu %ld %ld %ld %ld %ld %ld %ld",
               &user, &nice, &system, &idle, &iowait, &irq, &softirq);
        
        long total = user + nice + system + idle + iowait + irq + softirq;
        long total_diff = total - prev_total_;
        long idle_diff = idle - prev_idle_;
        
        if (total_diff > 0) {
            metrics.cpu_percent = 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff);
        }
        
        prev_total_ = total;
        prev_idle_ = idle;
        
        // Memory from /proc/self/status
        std::ifstream status_file("/proc/self/status");
        std::string key;
        while (status_file >> key) {
            if (key == "VmSize:") {
                status_file >> metrics.memory_bytes;
                metrics.memory_bytes *= 1024;  // Convert from KB
            } else if (key == "VmRSS:") {
                status_file >> metrics.rss_bytes;
                metrics.rss_bytes *= 1024;
            }
        }
        
#elif __APPLE__
        // CPU usage using Mach APIs
        mach_msg_type_number_t count = HOST_CPU_LOAD_INFO_COUNT;
        host_cpu_load_info_data_t cpu_load;
        
        if (host_statistics(mach_host_self(), HOST_CPU_LOAD_INFO,
                           (host_info_t)&cpu_load, &count) == KERN_SUCCESS) {
            uint64_t user = cpu_load.cpu_ticks[CPU_STATE_USER];
            uint64_t system_ticks = cpu_load.cpu_ticks[CPU_STATE_SYSTEM];
            uint64_t idle = cpu_load.cpu_ticks[CPU_STATE_IDLE];
            uint64_t nice = cpu_load.cpu_ticks[CPU_STATE_NICE];
            
            uint64_t total = user + system_ticks + idle + nice;
            uint64_t total_diff = total - prev_total_;
            uint64_t idle_diff = idle - prev_idle_;
            
            if (total_diff > 0) {
                metrics.cpu_percent = 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff);
            }
            
            prev_total_ = total;
            prev_idle_ = idle;
        }
        
        // Memory using task_info
        task_basic_info_data_t task_info_data;
        mach_msg_type_number_t task_info_count = TASK_BASIC_INFO_COUNT;
        
        if (task_info(mach_task_self(), TASK_BASIC_INFO,
                     (task_info_t)&task_info_data, &task_info_count) == KERN_SUCCESS) {
            metrics.memory_bytes = task_info_data.virtual_size;
            metrics.rss_bytes = task_info_data.resident_size;
        }
#endif
        
        return metrics;
    }
    
    SystemMetrics current_;
    std::vector<SystemMetrics> samples_;
    size_t max_samples_ = 1000;
    
    uint64_t prev_total_ = 0;
    uint64_t prev_idle_ = 0;
};

// =============================================================================
// Message Statistics
// =============================================================================

struct TopicStats {
    std::string topic;
    uint64_t expected_count = 0;
    std::atomic<uint64_t> received_count{0};
    std::atomic<uint64_t> total_bytes{0};
    std::vector<double> latencies_ms;
    std::mutex latency_mutex;
    
    uint64_t sequence = 0;
    uint64_t last_sequence = 0;
    uint64_t dropped_count = 0;
    
    void add_latency(double ms) {
        std::lock_guard<std::mutex> lock(latency_mutex);
        latencies_ms.push_back(ms);
    }
    
    double drop_rate() const {
        if (expected_count == 0) return 0.0;
        return 100.0 * static_cast<double>(expected_count - received_count.load()) / expected_count;
    }
    
    double avg_latency() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(latency_mutex));
        if (latencies_ms.empty()) return 0.0;
        return std::accumulate(latencies_ms.begin(), latencies_ms.end(), 0.0) / latencies_ms.size();
    }
    
    double p99_latency() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(latency_mutex));
        if (latencies_ms.empty()) return 0.0;
        
        std::vector<double> sorted = latencies_ms;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(sorted.size() * 0.99);
        return sorted[std::min(idx, sorted.size() - 1)];
    }
};

// =============================================================================
// Performance Test Results
// =============================================================================

struct PerformanceResults {
    double duration_sec = 0;
    double avg_cpu_percent = 0;
    double peak_cpu_percent = 0;
    size_t peak_memory_mb = 0;
    
    std::map<std::string, TopicStats*> topic_stats;
    
    bool cpu_target_met = false;   // <10% CPU
    bool drop_target_met = false;  // 0% drops
    
    void evaluate_targets() {
        cpu_target_met = peak_cpu_percent < 10.0;
        
        drop_target_met = true;
        for (const auto& [name, stats] : topic_stats) {
            if (stats->drop_rate() > 0.0) {
                drop_target_met = false;
                break;
            }
        }
    }
    
    void print() const {
        std::cout << "\n"
                  << "╔══════════════════════════════════════════════════════════╗\n"
                  << "║           PERFORMANCE TEST RESULTS                       ║\n"
                  << "╠══════════════════════════════════════════════════════════╣\n";
        
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "║ Duration: " << std::setw(8) << duration_sec << " seconds"
                  << std::string(30, ' ') << "║\n";
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";
        std::cout << "║ SYSTEM METRICS                                           ║\n";
        std::cout << "║   Average CPU: " << std::setw(6) << avg_cpu_percent << "%"
                  << "    Target: <10%  " << (cpu_target_met ? "[PASS]" : "[FAIL]") 
                  << std::string(6, ' ') << "║\n";
        std::cout << "║   Peak CPU:    " << std::setw(6) << peak_cpu_percent << "%"
                  << std::string(25, ' ') << "║\n";
        std::cout << "║   Peak Memory: " << std::setw(6) << peak_memory_mb << " MB"
                  << std::string(24, ' ') << "║\n";
        
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";
        std::cout << "║ MESSAGE STATISTICS                                       ║\n";
        
        for (const auto& [name, stats] : topic_stats) {
            std::cout << "║ " << std::left << std::setw(20) << name.substr(0, 20)
                      << std::right << "                                       ║\n";
            std::cout << "║   Received: " << std::setw(8) << stats->received_count.load()
                      << " / " << std::setw(8) << stats->expected_count
                      << "  Drop: " << std::setw(5) << stats->drop_rate() << "%"
                      << std::string(4, ' ') << "║\n";
            std::cout << "║   Latency avg: " << std::setw(6) << stats->avg_latency() << " ms"
                      << "  P99: " << std::setw(6) << stats->p99_latency() << " ms"
                      << std::string(9, ' ') << "║\n";
        }
        
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";
        std::cout << "║ OVERALL: " 
                  << (cpu_target_met && drop_target_met ? "PASS ✓" : "FAIL ✗")
                  << std::string(44, ' ') << "║\n";
        std::cout << "╚══════════════════════════════════════════════════════════╝\n";
    }
    
    void save_json(const std::string& path) const {
        std::ofstream f(path);
        f << "{\n"
          << "  \"duration_sec\": " << duration_sec << ",\n"
          << "  \"avg_cpu_percent\": " << avg_cpu_percent << ",\n"
          << "  \"peak_cpu_percent\": " << peak_cpu_percent << ",\n"
          << "  \"peak_memory_mb\": " << peak_memory_mb << ",\n"
          << "  \"cpu_target_met\": " << (cpu_target_met ? "true" : "false") << ",\n"
          << "  \"drop_target_met\": " << (drop_target_met ? "true" : "false") << ",\n"
          << "  \"topics\": {\n";
        
        bool first = true;
        for (const auto& [name, stats] : topic_stats) {
            if (!first) f << ",\n";
            first = false;
            f << "    \"" << name << "\": {\n"
              << "      \"received\": " << stats->received_count.load() << ",\n"
              << "      \"expected\": " << stats->expected_count << ",\n"
              << "      \"drop_rate\": " << stats->drop_rate() << ",\n"
              << "      \"avg_latency_ms\": " << stats->avg_latency() << ",\n"
              << "      \"p99_latency_ms\": " << stats->p99_latency() << "\n"
              << "    }";
        }
        
        f << "\n  }\n}\n";
    }
};

// Test configuration
struct TestConfig {
    int duration_sec = 10;
    int imu_rate = 1000;
    int camera_rate = 30;
    int num_cameras = 3;
    std::string output_json;
    
    void parse_args(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
                duration_sec = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--imu-rate") == 0 && i + 1 < argc) {
                imu_rate = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--camera-rate") == 0 && i + 1 < argc) {
                camera_rate = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--num-cameras") == 0 && i + 1 < argc) {
                num_cameras = std::atoi(argv[++i]);
            } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
                output_json = argv[++i];
            }
        }
    }
};

} // namespace test
} // namespace axon

#if defined(AXON_ROS1)
// =============================================================================
// ROS 1 Performance Test
// =============================================================================

int main(int argc, char** argv) {
    ros::init(argc, argv, "perf_test_node");
    ros::NodeHandle nh;
    
    axon::test::TestConfig config;
    config.parse_args(argc, argv);
    
    axon::test::SystemMonitor monitor;
    axon::test::PerformanceResults results;
    
    // Create topic stats
    axon::test::TopicStats imu_stats;
    imu_stats.topic = "/imu/data";
    imu_stats.expected_count = config.duration_sec * config.imu_rate;
    
    std::vector<axon::test::TopicStats> rgb_stats(config.num_cameras);
    std::vector<axon::test::TopicStats> depth_stats(config.num_cameras);
    
    for (int i = 0; i < config.num_cameras; ++i) {
        rgb_stats[i].topic = "/camera" + std::to_string(i) + "/rgb";
        rgb_stats[i].expected_count = config.duration_sec * config.camera_rate;
        
        depth_stats[i].topic = "/camera" + std::to_string(i) + "/depth";
        depth_stats[i].expected_count = config.duration_sec * config.camera_rate;
    }
    
    // Subscribe to topics
    auto imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000,
        [&](const sensor_msgs::Imu::ConstPtr& msg) {
            double latency = (ros::Time::now() - msg->header.stamp).toSec() * 1000.0;
            imu_stats.received_count++;
            imu_stats.add_latency(latency);
        });
    
    std::vector<ros::Subscriber> rgb_subs, depth_subs;
    for (int i = 0; i < config.num_cameras; ++i) {
        rgb_subs.push_back(nh.subscribe<sensor_msgs::Image>(
            "/camera" + std::to_string(i) + "/rgb", 100,
            [&rgb_stats, i](const sensor_msgs::Image::ConstPtr& msg) {
                double latency = (ros::Time::now() - msg->header.stamp).toSec() * 1000.0;
                rgb_stats[i].received_count++;
                rgb_stats[i].total_bytes += msg->data.size();
                rgb_stats[i].add_latency(latency);
            }));
        
        depth_subs.push_back(nh.subscribe<sensor_msgs::Image>(
            "/camera" + std::to_string(i) + "/depth", 100,
            [&depth_stats, i](const sensor_msgs::Image::ConstPtr& msg) {
                double latency = (ros::Time::now() - msg->header.stamp).toSec() * 1000.0;
                depth_stats[i].received_count++;
                depth_stats[i].total_bytes += msg->data.size();
                depth_stats[i].add_latency(latency);
            }));
    }
    
    std::cout << "Performance test starting...\n"
              << "Duration: " << config.duration_sec << " seconds\n"
              << "Press Ctrl+C to stop early\n";
    
    auto start = std::chrono::steady_clock::now();
    ros::Rate rate(10);  // 10Hz monitoring
    
    while (ros::ok()) {
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start).count();
        
        if (elapsed >= config.duration_sec) break;
        
        monitor.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    // Collect results
    auto end = std::chrono::steady_clock::now();
    results.duration_sec = std::chrono::duration<double>(end - start).count();
    results.avg_cpu_percent = monitor.average().cpu_percent;
    results.peak_cpu_percent = monitor.peak_cpu();
    results.peak_memory_mb = monitor.peak_memory() / 1024 / 1024;
    
    results.topic_stats[imu_stats.topic] = &imu_stats;
    for (int i = 0; i < config.num_cameras; ++i) {
        results.topic_stats[rgb_stats[i].topic] = &rgb_stats[i];
        results.topic_stats[depth_stats[i].topic] = &depth_stats[i];
    }
    
    results.evaluate_targets();
    results.print();
    
    if (!config.output_json.empty()) {
        results.save_json(config.output_json);
        std::cout << "Results saved to: " << config.output_json << "\n";
    }
    
    return (results.cpu_target_met && results.drop_target_met) ? 0 : 1;
}

#elif defined(AXON_ROS2)
// =============================================================================
// ROS 2 Performance Test
// =============================================================================

class PerfTestNode : public rclcpp::Node {
public:
    PerfTestNode(const axon::test::TestConfig& config)
        : Node("perf_test_node")
        , config_(config)
    {
        // Initialize stats
        imu_stats_.topic = "/imu/data";
        imu_stats_.expected_count = config.duration_sec * config.imu_rate;
        
        rgb_stats_.resize(config.num_cameras);
        depth_stats_.resize(config.num_cameras);
        
        for (int i = 0; i < config.num_cameras; ++i) {
            rgb_stats_[i].topic = "/camera" + std::to_string(i) + "/rgb";
            rgb_stats_[i].expected_count = config.duration_sec * config.camera_rate;
            
            depth_stats_[i].topic = "/camera" + std::to_string(i) + "/depth";
            depth_stats_[i].expected_count = config.duration_sec * config.camera_rate;
        }
        
        // Subscribe to IMU
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 1000,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                auto now = this->now();
                double latency = (now - msg->header.stamp).seconds() * 1000.0;
                imu_stats_.received_count++;
                imu_stats_.add_latency(latency);
            });
        
        // Subscribe to cameras
        for (int i = 0; i < config.num_cameras; ++i) {
            rgb_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
                "/camera" + std::to_string(i) + "/rgb", 100,
                [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
                    auto now = this->now();
                    double latency = (now - msg->header.stamp).seconds() * 1000.0;
                    rgb_stats_[i].received_count++;
                    rgb_stats_[i].total_bytes += msg->data.size();
                    rgb_stats_[i].add_latency(latency);
                }));
            
            depth_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
                "/camera" + std::to_string(i) + "/depth", 100,
                [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
                    auto now = this->now();
                    double latency = (now - msg->header.stamp).seconds() * 1000.0;
                    depth_stats_[i].received_count++;
                    depth_stats_[i].total_bytes += msg->data.size();
                    depth_stats_[i].add_latency(latency);
                }));
        }
        
        // Monitoring timer
        monitor_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { monitor_.update(); });
        
        // Duration timer
        start_time_ = std::chrono::steady_clock::now();
        duration_timer_ = create_wall_timer(
            std::chrono::seconds(config.duration_sec),
            [this]() { finish_test(); });
        
        RCLCPP_INFO(get_logger(), "Performance test starting...");
    }
    
    void finish_test() {
        auto end = std::chrono::steady_clock::now();
        results_.duration_sec = std::chrono::duration<double>(end - start_time_).count();
        results_.avg_cpu_percent = monitor_.average().cpu_percent;
        results_.peak_cpu_percent = monitor_.peak_cpu();
        results_.peak_memory_mb = monitor_.peak_memory() / 1024 / 1024;
        
        results_.topic_stats[imu_stats_.topic] = &imu_stats_;
        for (size_t i = 0; i < rgb_stats_.size(); ++i) {
            results_.topic_stats[rgb_stats_[i].topic] = &rgb_stats_[i];
            results_.topic_stats[depth_stats_[i].topic] = &depth_stats_[i];
        }
        
        results_.evaluate_targets();
        results_.print();
        
        if (!config_.output_json.empty()) {
            results_.save_json(config_.output_json);
            RCLCPP_INFO(get_logger(), "Results saved to: %s", config_.output_json.c_str());
        }
        
        rclcpp::shutdown();
    }
    
    bool passed() const {
        return results_.cpu_target_met && results_.drop_target_met;
    }

private:
    axon::test::TestConfig config_;
    axon::test::SystemMonitor monitor_;
    axon::test::PerformanceResults results_;
    std::chrono::steady_clock::time_point start_time_;
    
    axon::test::TopicStats imu_stats_;
    std::vector<axon::test::TopicStats> rgb_stats_;
    std::vector<axon::test::TopicStats> depth_stats_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> rgb_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> depth_subs_;
    
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr duration_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    axon::test::TestConfig config;
    config.parse_args(argc, argv);
    
    auto node = std::make_shared<PerfTestNode>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return node->passed() ? 0 : 1;
}

#else
int main() {
    std::cout << "Compile with -DAXON_ROS1 or -DAXON_ROS2\n";
    return 1;
}
#endif

//! Performance Test Monitor
//!
//! This program monitors the axon_recorder's performance by:
//! - Finding the recorder process by PID
//! - Reading /proc/<pid>/stat for CPU usage
//! - Reading /proc/<pid>/status for memory usage
//! - Reading stats files written by publisher and recorder
//! - Calculating exact drop rate from message counts
//!
//! Key metrics:
//! - Recorder CPU usage (not system-wide)
//! - Recorder memory usage (RSS)
//! - Messages sent by publisher
//! - Messages received/dropped/written by recorder
//! - Exact drop rate calculation

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#if defined(AXON_ROS2)
#include <rclcpp/rclcpp.hpp>
#endif

namespace axon {
namespace test {

// =============================================================================
// Process-specific CPU and Memory Monitor
// =============================================================================

class ProcessMonitor {
public:
  ProcessMonitor() = default;

  // Find recorder process by name
  bool find_recorder_pid() {
    recorder_pid_ = find_pid_by_name("axon_recorder");
    if (recorder_pid_ > 0) {
      std::cout << "Found recorder process with PID: " << recorder_pid_ << "\n";
      // Initialize CPU tracking
      update();
      return true;
    }
    std::cerr << "Warning: Could not find axon_recorder process\n";
    return false;
  }

  void update() {
    if (recorder_pid_ <= 0) return;

    auto metrics = collect_process_metrics(recorder_pid_);
    samples_.push_back(metrics);

    // Keep only last N samples
    if (samples_.size() > max_samples_) {
      samples_.erase(samples_.begin());
    }
  }

  double average_cpu() const {
    if (samples_.size() < 2) return 0.0;
    double sum = 0.0;
    for (size_t i = 1; i < samples_.size(); ++i) {
      sum += samples_[i].cpu_percent;
    }
    return sum / (samples_.size() - 1);
  }

  double peak_cpu() const {
    double peak = 0.0;
    for (const auto& s : samples_) {
      peak = std::max(peak, s.cpu_percent);
    }
    return peak;
  }

  size_t peak_memory_mb() const {
    size_t peak = 0;
    for (const auto& s : samples_) {
      peak = std::max(peak, s.rss_bytes);
    }
    return peak / (1024 * 1024);
  }

  size_t current_memory_mb() const {
    if (samples_.empty()) return 0;
    return samples_.back().rss_bytes / (1024 * 1024);
  }

private:
  struct ProcessMetrics {
    double cpu_percent = 0.0;
    size_t rss_bytes = 0;
    uint64_t utime = 0;
    uint64_t stime = 0;
  };

  // Find PID by process name
  int find_pid_by_name(const std::string& name) {
    DIR* dir = opendir("/proc");
    if (!dir) return -1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      // Check if directory name is a number (PID)
      if (entry->d_type == DT_DIR) {
        bool is_pid = true;
        for (char* c = entry->d_name; *c; ++c) {
          if (!isdigit(*c)) {
            is_pid = false;
            break;
          }
        }

        if (is_pid) {
          int pid = std::atoi(entry->d_name);
          std::string cmdline_path = "/proc/" + std::string(entry->d_name) + "/cmdline";
          std::ifstream cmdline(cmdline_path);
          std::string cmd;
          std::getline(cmdline, cmd, '\0');

          if (cmd.find(name) != std::string::npos) {
            closedir(dir);
            return pid;
          }
        }
      }
    }
    closedir(dir);
    return -1;
  }

  ProcessMetrics collect_process_metrics(int pid) {
    ProcessMetrics metrics;

#ifdef __linux__
    // Read /proc/<pid>/stat for CPU times
    std::string stat_path = "/proc/" + std::to_string(pid) + "/stat";
    std::ifstream stat_file(stat_path);
    if (stat_file.is_open()) {
      std::string line;
      std::getline(stat_file, line);

      // Parse stat file - format: pid (comm) state ppid ... utime stime ...
      // Fields are space-separated, but comm can contain spaces
      size_t start = line.find(')');
      if (start != std::string::npos) {
        std::istringstream iss(line.substr(start + 2));
        std::string token;

        // Skip to utime (field 14) and stime (field 15) after comm
        // Fields after ')': state(1) ppid(2) pgrp(3) session(4) tty(5) tpgid(6)
        // flags(7) minflt(8) cminflt(9) majflt(10) cmajflt(11) utime(12) stime(13)
        for (int i = 0; i < 11 && iss >> token; ++i) {
        }

        uint64_t utime, stime;
        if (iss >> utime >> stime) {
          // Calculate CPU percentage based on delta
          uint64_t total_time = utime + stime;
          auto now = std::chrono::steady_clock::now();

          if (prev_time_.time_since_epoch().count() > 0) {
            double elapsed_sec = std::chrono::duration<double>(now - prev_time_).count();
            uint64_t time_delta = total_time - prev_cpu_time_;

            // CPU ticks are in clock ticks (usually 100 per second)
            long clk_tck = sysconf(_SC_CLK_TCK);
            double cpu_sec = static_cast<double>(time_delta) / clk_tck;
            metrics.cpu_percent = (cpu_sec / elapsed_sec) * 100.0;
          }

          metrics.utime = utime;
          metrics.stime = stime;
          prev_cpu_time_ = total_time;
          prev_time_ = now;
        }
      }
    }

    // Read /proc/<pid>/status for memory
    std::string status_path = "/proc/" + std::to_string(pid) + "/status";
    std::ifstream status_file(status_path);
    if (status_file.is_open()) {
      std::string line;
      while (std::getline(status_file, line)) {
        // Format: "VmRSS:\t   12345 kB"
        if (line.compare(0, 6, "VmRSS:") == 0) {
          // Extract the number - skip "VmRSS:" and any whitespace
          size_t pos = 6;
          while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t')) {
            ++pos;
          }
          size_t rss_kb = 0;
          while (pos < line.size() && isdigit(line[pos])) {
            rss_kb = rss_kb * 10 + (line[pos] - '0');
            ++pos;
          }
          if (rss_kb > 0) {
            metrics.rss_bytes = rss_kb * 1024;
          }
          break;
        }
      }
    }
#endif

    return metrics;
  }

  int recorder_pid_ = -1;
  std::vector<ProcessMetrics> samples_;
  size_t max_samples_ = 1000;

  uint64_t prev_cpu_time_ = 0;
  std::chrono::steady_clock::time_point prev_time_;
};

// =============================================================================
// Stats File Reader
// =============================================================================

struct PublisherStats {
  uint64_t imu_published = 0;
  uint64_t rgb_published = 0;
  uint64_t depth_published = 0;
  uint64_t total_messages = 0;
  uint64_t total_bytes = 0;
  double duration_sec = 0.0;
  bool loaded = false;

  bool load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cerr << "Could not open publisher stats: " << path << "\n";
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      // Simple JSON parsing
      if (line.find("\"imu_published\"") != std::string::npos) {
        sscanf(line.c_str(), " \"imu_published\": %lu", &imu_published);
      } else if (line.find("\"rgb_published\"") != std::string::npos) {
        sscanf(line.c_str(), " \"rgb_published\": %lu", &rgb_published);
      } else if (line.find("\"depth_published\"") != std::string::npos) {
        sscanf(line.c_str(), " \"depth_published\": %lu", &depth_published);
      } else if (line.find("\"total_messages\"") != std::string::npos) {
        sscanf(line.c_str(), " \"total_messages\": %lu", &total_messages);
      } else if (line.find("\"total_bytes\"") != std::string::npos) {
        sscanf(line.c_str(), " \"total_bytes\": %lu", &total_bytes);
      } else if (line.find("\"duration_sec\"") != std::string::npos) {
        sscanf(line.c_str(), " \"duration_sec\": %lf", &duration_sec);
      }
    }

    loaded = true;
    return true;
  }
};

struct RecorderStats {
  uint64_t messages_received = 0;
  uint64_t messages_dropped = 0;
  uint64_t messages_written = 0;
  double drop_rate_percent = 0.0;
  bool loaded = false;

  bool load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cerr << "Could not open recorder stats: " << path << "\n";
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("\"messages_received\"") != std::string::npos) {
        sscanf(line.c_str(), " \"messages_received\": %lu", &messages_received);
      } else if (line.find("\"messages_dropped\"") != std::string::npos) {
        sscanf(line.c_str(), " \"messages_dropped\": %lu", &messages_dropped);
      } else if (line.find("\"messages_written\"") != std::string::npos) {
        sscanf(line.c_str(), " \"messages_written\": %lu", &messages_written);
      } else if (line.find("\"drop_rate_percent\"") != std::string::npos) {
        sscanf(line.c_str(), " \"drop_rate_percent\": %lf", &drop_rate_percent);
      }
    }

    loaded = true;
    return true;
  }
};

// =============================================================================
// Test Configuration
// =============================================================================

struct TestConfig {
  int duration_sec = 10;
  std::string publisher_stats_path = "/data/recordings/publisher_stats.json";
  std::string recorder_stats_path = "/data/recordings/recorder_stats.json";
  std::string output_json;

  void parse_args(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
      if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
        duration_sec = std::atoi(argv[++i]);
      } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
        output_json = argv[++i];
      }
    }
  }
};

// =============================================================================
// Results Reporting
// =============================================================================

struct PerformanceResults {
  double duration_sec = 0;
  double avg_cpu_percent = 0;
  double peak_cpu_percent = 0;
  size_t peak_memory_mb = 0;

  uint64_t messages_sent = 0;
  uint64_t messages_received = 0;
  uint64_t messages_dropped = 0;
  uint64_t messages_written = 0;
  double drop_rate_percent = 0;

  bool cpu_target_met = false;   // <50% CPU for recorder
  bool drop_target_met = false;  // <5% drops

  void calculate_drop_rate() {
    if (messages_sent > 0) {
      // Handle case where recorder received more than publisher reported
      // (can happen due to timing differences in stat collection)
      if (messages_written >= messages_sent) {
        drop_rate_percent = 0.0;  // No drops - everything was received
      } else {
        uint64_t lost = messages_sent - messages_written;
        drop_rate_percent = 100.0 * static_cast<double>(lost) / static_cast<double>(messages_sent);
      }
    }
  }

  void evaluate_targets() {
    cpu_target_met = avg_cpu_percent < 50.0;
    drop_target_met = drop_rate_percent < 5.0;
  }

  void print() const {
    // Table width (content between ║ characters)
    constexpr int WIDTH = 58;

    // Helper to create a padded row
    auto row = [](const std::string& content) {
      std::ostringstream oss;
      oss << "║ " << std::left << std::setw(WIDTH - 1) << content << "║\n";
      return oss.str();
    };

    // Helper for key-value rows with optional status
    auto kv_row = [&](
                    const std::string& label,
                    const std::string& value,
                    const std::string& target = "",
                    bool* passed = nullptr
                  ) {
      std::ostringstream oss;
      oss << "║   " << std::left << std::setw(22) << label << std::right << std::setw(12) << value;
      if (!target.empty() && passed) {
        oss << "  " << std::left << std::setw(12) << target
            << (*passed ? "\033[32m[PASS]\033[0m" : "\033[31m[FAIL]\033[0m");
      } else {
        oss << std::string(20, ' ');
      }
      oss << " ║\n";
      return oss.str();
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);

    // Header
    out << "\n";
    out << "╔══════════════════════════════════════════════════════════╗\n";
    out << row("          PERFORMANCE TEST RESULTS");
    out << "╠══════════════════════════════════════════════════════════╣\n";

    // Duration
    std::ostringstream dur;
    dur << std::fixed << std::setprecision(2) << duration_sec << " seconds";
    out << "║   " << std::left << std::setw(22) << "Duration:" << std::right << std::setw(12)
        << dur.str() << std::string(20, ' ') << "║\n";

    // Recorder metrics section
    out << "╠══════════════════════════════════════════════════════════╣\n";
    out << row("RECORDER PROCESS METRICS");

    std::ostringstream cpu_val, peak_cpu_val, mem_val;
    cpu_val << std::fixed << std::setprecision(2) << avg_cpu_percent << "%";
    peak_cpu_val << std::fixed << std::setprecision(2) << peak_cpu_percent << "%";
    mem_val << peak_memory_mb << " MB";

    bool cpu_pass = cpu_target_met;
    out << kv_row("Average CPU:", cpu_val.str(), "Target: <50%", &cpu_pass);
    out << kv_row("Peak CPU:", peak_cpu_val.str());
    out << kv_row("Peak Memory:", mem_val.str());

    // Message statistics section
    out << "╠══════════════════════════════════════════════════════════╣\n";
    out << row("MESSAGE STATISTICS");

    out << kv_row("Sent by Publisher:", std::to_string(messages_sent));
    out << kv_row("Received by Recorder:", std::to_string(messages_received));
    out << kv_row("Dropped (queue full):", std::to_string(messages_dropped));
    out << kv_row("Written to Lance:", std::to_string(messages_written));

    std::ostringstream drop_val;
    drop_val << std::fixed << std::setprecision(2) << drop_rate_percent << "%";
    bool drop_pass = drop_target_met;
    out << kv_row("Drop Rate:", drop_val.str(), "Target: <5%", &drop_pass);

    // Overall result
    out << "╠══════════════════════════════════════════════════════════╣\n";
    bool passed = cpu_target_met && drop_target_met;
    std::string result_str = passed ? "\033[32mPASS ✓\033[0m" : "\033[31mFAIL ✗\033[0m";
    out << "║   OVERALL: " << result_str << std::string(39, ' ') << " ║\n";
    out << "╚══════════════════════════════════════════════════════════╝\n";

    std::cout << out.str();
  }

  void save_json(const std::string& path) const {
    std::ofstream f(path);
    f << "{\n"
      << "  \"duration_sec\": " << duration_sec << ",\n"
      << "  \"avg_cpu_percent\": " << avg_cpu_percent << ",\n"
      << "  \"peak_cpu_percent\": " << peak_cpu_percent << ",\n"
      << "  \"peak_memory_mb\": " << peak_memory_mb << ",\n"
      << "  \"messages_sent\": " << messages_sent << ",\n"
      << "  \"messages_received\": " << messages_received << ",\n"
      << "  \"messages_dropped\": " << messages_dropped << ",\n"
      << "  \"messages_written\": " << messages_written << ",\n"
      << "  \"drop_rate_percent\": " << drop_rate_percent << ",\n"
      << "  \"cpu_target_met\": " << (cpu_target_met ? "true" : "false") << ",\n"
      << "  \"drop_target_met\": " << (drop_target_met ? "true" : "false") << "\n"
      << "}\n";
  }
};

}  // namespace test
}  // namespace axon

// =============================================================================
// Main Entry Point
// =============================================================================

#if defined(AXON_ROS1)

int main(int argc, char** argv) {
  std::cout << "ROS1 performance monitor not yet implemented\n";
  return 1;
}

#elif defined(AXON_ROS2)

class PerfMonitorNode : public rclcpp::Node {
public:
  PerfMonitorNode(const axon::test::TestConfig& config)
      : Node("perf_monitor")
      , config_(config) {
    RCLCPP_INFO(get_logger(), "Performance Monitor starting...");
    RCLCPP_INFO(get_logger(), "Will monitor for %d seconds", config_.duration_sec);

    // Try to find recorder PID
    if (!monitor_.find_recorder_pid()) {
      RCLCPP_WARN(get_logger(), "Recorder not found yet, will retry...");
    }

    // Monitoring timer - sample every 500ms
    start_time_ = std::chrono::steady_clock::now();
    monitor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      // Try to find recorder if not found yet
      if (!found_recorder_) {
        found_recorder_ = monitor_.find_recorder_pid();
      }

      monitor_.update();

      // Log progress
      auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

      RCLCPP_INFO(
        get_logger(),
        "Progress: %.1fs, Recorder CPU: %.1f%%, Memory: %zu MB",
        elapsed,
        monitor_.average_cpu(),
        monitor_.current_memory_mb()
      );
    });

    // Duration timer - wait for test to complete, then read stats
    // Add extra time for: publisher to finish, recorder to flush and write stats
    duration_timer_ = create_wall_timer(
      std::chrono::seconds(config_.duration_sec + 15),  // +15s for proper shutdown sequence
      [this]() {
        finish_test();
      }
    );
  }

  void finish_test() {
    RCLCPP_INFO(get_logger(), "Test duration complete, collecting results...");

    auto end_time = std::chrono::steady_clock::now();
    results_.duration_sec = std::chrono::duration<double>(end_time - start_time_).count();

    // Get CPU/memory metrics
    results_.avg_cpu_percent = monitor_.average_cpu();
    results_.peak_cpu_percent = monitor_.peak_cpu();
    results_.peak_memory_mb = monitor_.peak_memory_mb();

    // Load stats files with retry (recorder may still be writing)
    axon::test::PublisherStats pub_stats;
    axon::test::RecorderStats rec_stats;

    // Retry loading stats files (up to 10 seconds)
    for (int attempt = 0; attempt < 20; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      if (!pub_stats.loaded) {
        pub_stats.load(config_.publisher_stats_path);
      }
      if (!rec_stats.loaded) {
        rec_stats.load(config_.recorder_stats_path);
      }

      if (pub_stats.loaded && rec_stats.loaded) {
        break;
      }

      if (attempt % 4 == 0) {
        RCLCPP_INFO(get_logger(), "Waiting for stats files... (attempt %d)", attempt + 1);
      }
    }

    if (pub_stats.loaded) {
      results_.messages_sent = pub_stats.total_messages;
      RCLCPP_INFO(get_logger(), "Publisher stats: %lu messages sent", pub_stats.total_messages);
    } else {
      RCLCPP_WARN(get_logger(), "Could not load publisher stats after retries");
    }

    if (rec_stats.loaded) {
      results_.messages_received = rec_stats.messages_received;
      results_.messages_dropped = rec_stats.messages_dropped;
      results_.messages_written = rec_stats.messages_written;
      RCLCPP_INFO(
        get_logger(),
        "Recorder stats: received=%lu, dropped=%lu, written=%lu",
        rec_stats.messages_received,
        rec_stats.messages_dropped,
        rec_stats.messages_written
      );
    } else {
      RCLCPP_WARN(get_logger(), "Could not load recorder stats after retries");
    }

    // Calculate drop rate
    results_.calculate_drop_rate();
    results_.evaluate_targets();

    // Print and save results
    results_.print();

    if (!config_.output_json.empty()) {
      results_.save_json(config_.output_json);
      RCLCPP_INFO(get_logger(), "Results saved to: %s", config_.output_json.c_str());
    }

    passed_ = results_.cpu_target_met && results_.drop_target_met;
    rclcpp::shutdown();
  }

  bool passed() const {
    return passed_;
  }

private:
  axon::test::TestConfig config_;
  axon::test::ProcessMonitor monitor_;
  axon::test::PerformanceResults results_;
  std::chrono::steady_clock::time_point start_time_;
  bool found_recorder_ = false;
  bool passed_ = false;

  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr duration_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  axon::test::TestConfig config;
  config.parse_args(argc, argv);

  auto node = std::make_shared<PerfMonitorNode>(config);

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

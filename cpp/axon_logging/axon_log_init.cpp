#include "axon_log_init.hpp"
#include "axon_log_macros.hpp"
#include <boost/log/core.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/make_shared.hpp>
#include <algorithm>
#include <vector>
#include <mutex>

namespace axon {
namespace logging {

namespace {
    // Thread safety for sink management
    std::mutex g_sinks_mutex;
    
    // Store sink pointers for cleanup
    std::vector<boost::shared_ptr<boost::log::sinks::sink>> g_sinks;
    boost::shared_ptr<async_console_sink_t> g_console_sink;
    boost::shared_ptr<async_file_sink_t> g_file_sink;
    
    // Global logger instance (Meyers singleton)
    logger_type& get_logger_impl() {
        static logger_type instance;
        return instance;
    }
    
    bool g_initialized = false;
}

logger_type& get_logger() {
    return get_logger_impl();
}

void init_logging(const LoggingConfig& config) {
    std::lock_guard<std::mutex> lock(g_sinks_mutex);
    
    if (g_initialized) {
        return;  // Already initialized
    }
    
    auto core = boost::log::core::get();
    
    // Add common attributes (TimeStamp, ThreadID, etc.)
    boost::log::add_common_attributes();
    
    // Console sink
    if (config.console_enabled) {
        g_console_sink = create_console_sink(config.console_level, config.console_colors);
        core->add_sink(g_console_sink);
        g_sinks.push_back(g_console_sink);
    }
    
    // File sink
    if (config.file_enabled) {
        g_file_sink = create_file_sink(config.file_config, config.file_level);
        core->add_sink(g_file_sink);
        g_sinks.push_back(g_file_sink);
    }
    
    g_initialized = true;
}

void init_logging_default() {
    LoggingConfig config;
    config.console_enabled = true;
    config.console_colors = true;
    config.console_level = severity_level::info;
    config.file_enabled = false;  // Disabled by default
    
    init_logging(config);
}

void shutdown_logging() {
    std::lock_guard<std::mutex> lock(g_sinks_mutex);
    
    if (!g_initialized) {
        return;
    }
    
    auto core = boost::log::core::get();
    
    // Stop async sinks first (this drains the queue)
    if (g_console_sink) {
        g_console_sink->stop();
        g_console_sink->flush();
    }
    if (g_file_sink) {
        g_file_sink->stop();
        g_file_sink->flush();
    }
    
    // Remove all sinks
    for (auto& sink : g_sinks) {
        core->remove_sink(sink);
    }
    g_sinks.clear();
    
    g_console_sink.reset();
    g_file_sink.reset();
    
    g_initialized = false;
}

void add_sink(boost::shared_ptr<boost::log::sinks::sink> sink) {
    std::lock_guard<std::mutex> lock(g_sinks_mutex);
    
    auto core = boost::log::core::get();
    core->add_sink(sink);
    g_sinks.push_back(sink);
}

void remove_sink(boost::shared_ptr<boost::log::sinks::sink> sink) {
    std::lock_guard<std::mutex> lock(g_sinks_mutex);
    
    auto core = boost::log::core::get();
    core->remove_sink(sink);
    
    // Remove from our tracking vector
    auto it = std::find(g_sinks.begin(), g_sinks.end(), sink);
    if (it != g_sinks.end()) {
        g_sinks.erase(it);
    }
}

void flush_logging() {
    std::lock_guard<std::mutex> lock(g_sinks_mutex);
    
    if (g_console_sink) {
        g_console_sink->flush();
    }
    if (g_file_sink) {
        g_file_sink->flush();
    }
}

}  // namespace logging
}  // namespace axon


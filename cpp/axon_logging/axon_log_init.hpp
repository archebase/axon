#ifndef AXON_LOG_INIT_HPP
#define AXON_LOG_INIT_HPP

#include "axon_log_severity.hpp"
#include "axon_console_sink.hpp"
#include "axon_file_sink.hpp"
#include <boost/log/sinks/sink.hpp>

namespace axon {
namespace logging {

/**
 * Logging configuration for axon applications.
 * This covers console and file sinks (core logging).
 * ROS sink is configured separately in ros/axon_recorder/.
 */
struct LoggingConfig {
    // Console sink
    bool console_enabled = true;
    bool console_colors = true;
    severity_level console_level = severity_level::info;
    
    // File sink
    bool file_enabled = true;
    FileSinkConfig file_config;  // Uses FileSinkConfig defaults
    severity_level file_level = severity_level::debug;
};

/**
 * Initialize core logging (console and file sinks).
 * Should be called once at application startup.
 * 
 * ROS sink should be added separately via add_sink() after ROS is initialized.
 * 
 * @param config Logging configuration
 */
void init_logging(const LoggingConfig& config);

/**
 * Initialize with default configuration.
 * Console: INFO level, colors enabled
 * File: disabled (set file_enabled=true and provide directory)
 */
void init_logging_default();

/**
 * Shutdown logging infrastructure.
 * Stops async sink threads and flushes all pending records.
 * Should be called before application exit.
 */
void shutdown_logging();

/**
 * Add a custom sink to the logging core.
 * Use this to add ROS sink or other custom sinks.
 * 
 * @param sink The sink to add
 */
void add_sink(boost::shared_ptr<boost::log::sinks::sink> sink);

/**
 * Remove a sink from the logging core.
 * 
 * @param sink The sink to remove
 */
void remove_sink(boost::shared_ptr<boost::log::sinks::sink> sink);

/**
 * Flush all sinks.
 */
void flush_logging();

}  // namespace logging
}  // namespace axon

#endif  // AXON_LOG_INIT_HPP

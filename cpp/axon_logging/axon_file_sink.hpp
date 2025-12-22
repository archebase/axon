#ifndef AXON_FILE_SINK_HPP
#define AXON_FILE_SINK_HPP

#include "axon_log_severity.hpp"
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/bounded_fifo_queue.hpp>
#include <boost/log/sinks/drop_on_overflow.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <string>

namespace axon {
namespace logging {

/**
 * Async file sink with bounded queue.
 * Uses larger queue than console sink for slower I/O.
 */
typedef boost::log::sinks::asynchronous_sink<
    boost::log::sinks::text_file_backend,
    boost::log::sinks::bounded_fifo_queue<
        5000,  // Larger queue for file sink (slower I/O)
        boost::log::sinks::drop_on_overflow
    >
> async_file_sink_t;

/**
 * File sink configuration.
 */
struct FileSinkConfig {
    std::string directory = "/var/log/axon";
    std::string file_pattern = "recorder_%Y%m%d_%H%M%S.log";
    uint64_t rotation_size_mb = 100;      // Rotate at 100MB
    bool rotate_at_midnight = true;       // Also rotate daily
    int max_files = 10;                   // Keep 10 rotated files
    bool format_json = true;              // JSON format for log aggregation
};

/**
 * Create async file sink with rotation.
 * - Size-based rotation (default 100MB)
 * - Time-based rotation (midnight)
 * - Bounded queue prevents memory explosion
 * - JSON format for log aggregation tools
 * 
 * @param config File sink configuration
 * @param min_level Minimum severity level to log
 * @return Shared pointer to the sink
 */
boost::shared_ptr<async_file_sink_t> create_file_sink(
    const FileSinkConfig& config,
    severity_level min_level = severity_level::debug
);

}  // namespace logging
}  // namespace axon

#endif  // AXON_FILE_SINK_HPP

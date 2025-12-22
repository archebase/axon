#ifndef AXON_LOG_MACROS_HPP
#define AXON_LOG_MACROS_HPP

#include "axon_log_severity.hpp"
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/attributes/constant.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <sstream>

namespace axon {
namespace logging {

// Global severity logger type
typedef boost::log::sources::severity_logger<severity_level> logger_type;

/**
 * Get the global logger instance.
 * Defined in axon_log_init.cpp
 */
logger_type& get_logger();

/**
 * Simple key-value formatter for structured logging.
 * Returns a string that can be streamed to log output.
 * Usage: AXON_LOG_INFO("message" << kv("key", value));
 */
template<typename T>
inline std::string kv(const char* name, const T& value) {
    std::ostringstream oss;
    oss << " " << name << "=" << value;
    return oss.str();
}

// Specialization for std::string to add quotes
template<>
inline std::string kv(const char* name, const std::string& value) {
    std::ostringstream oss;
    oss << " " << name << "=\"" << value << "\"";
    return oss.str();
}

// Specialization for const char* to add quotes
inline std::string kv(const char* name, const char* value) {
    std::ostringstream oss;
    oss << " " << name << "=\"" << value << "\"";
    return oss.str();
}

}  // namespace logging
}  // namespace axon

// =============================================================================
// Component identification
// Define AXON_LOG_COMPONENT before including this header to set component name.
//
// Example:
//   #define AXON_LOG_COMPONENT "my_component"
//   #include <axon_log_macros.hpp>
//
// If not defined, defaults to "axon" which makes it harder to trace log sources.
// Best practice: Define AXON_LOG_COMPONENT in each compilation unit that uses logging.
// =============================================================================
#ifndef AXON_LOG_COMPONENT
#define AXON_LOG_COMPONENT "axon"
#endif

// =============================================================================
// Compile-time debug toggle
// In release builds (NDEBUG defined), DEBUG logs are compiled out
// =============================================================================
#ifdef NDEBUG
#define AXON_LOG_ENABLE_DEBUG 0
#else
#define AXON_LOG_ENABLE_DEBUG 1
#endif

// =============================================================================
// Main logging macros - stream-based API
// Component is added as a prefix to the message for simplicity
//
// Usage: AXON_LOG_INFO("message" << kv("key", value));
// All macros use do-while(0) pattern for statement safety.
// =============================================================================

#define AXON_LOG_DEBUG(msg) \
    do { \
        if (AXON_LOG_ENABLE_DEBUG) { \
            BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::debug) \
                << "[" << AXON_LOG_COMPONENT << "] " << msg; \
        } \
    } while(0)

#define AXON_LOG_INFO(msg) \
    do { \
        BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::info) \
            << "[" << AXON_LOG_COMPONENT << "] " << msg; \
    } while(0)

#define AXON_LOG_WARN(msg) \
    do { \
        BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::warn) \
            << "[" << AXON_LOG_COMPONENT << "] " << msg; \
    } while(0)

#define AXON_LOG_ERROR(msg) \
    do { \
        BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::error) \
            << "[" << AXON_LOG_COMPONENT << "] " << msg; \
    } while(0)

#define AXON_LOG_FATAL(msg) \
    do { \
        BOOST_LOG_SEV(::axon::logging::get_logger(), ::axon::logging::severity_level::fatal) \
            << "[" << AXON_LOG_COMPONENT << "] " << msg; \
    } while(0)

// =============================================================================
// Context management using scoped attributes
// Usage: AXON_LOG_SCOPED_CONTEXT("task_123", "robot_001");
// Context is automatically cleared when the scope exits
// =============================================================================
#define AXON_LOG_SCOPED_CONTEXT(task_id_val, device_id_val) \
    BOOST_LOG_SCOPED_THREAD_ATTR("TaskID", \
        boost::log::attributes::constant<std::string>(task_id_val)); \
    BOOST_LOG_SCOPED_THREAD_ATTR("DeviceID", \
        boost::log::attributes::constant<std::string>(device_id_val))

#endif  // AXON_LOG_MACROS_HPP

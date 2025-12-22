#ifndef AXON_LOG_SEVERITY_HPP
#define AXON_LOG_SEVERITY_HPP

#include <boost/log/expressions/keyword.hpp>
#include <ostream>

namespace axon {
namespace logging {

/**
 * Severity levels for axon logging.
 * Maps to standard log levels with FATAL for unrecoverable errors.
 */
enum class severity_level {
    debug = 0,
    info = 1,
    warn = 2,
    error = 3,
    fatal = 4
};

// Output operator for formatting
inline std::ostream& operator<<(std::ostream& strm, severity_level level) {
    static const char* strings[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
    if (static_cast<size_t>(level) < sizeof(strings) / sizeof(*strings))
        strm << strings[static_cast<size_t>(level)];
    else
        strm << static_cast<int>(level);
    return strm;
}

// Boost.Log keyword for severity filtering
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level)

}  // namespace logging
}  // namespace axon

#endif  // AXON_LOG_SEVERITY_HPP

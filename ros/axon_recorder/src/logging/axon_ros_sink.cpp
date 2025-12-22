#include "axon_ros_sink.hpp"
#include <boost/log/expressions.hpp>
#include <boost/log/attributes/value_extraction.hpp>
#include <boost/make_shared.hpp>
#include <cassert>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;

ros_backend::ros_backend(recorder::RosInterface* ros_interface)
    : ros_interface_(ros_interface) {
    // ROS interface must outlive this sink. The caller is responsible for
    // ensuring the RosInterface pointer remains valid for the lifetime of this sink.
    // Typically, the sink should be removed before RosInterface is destroyed.
    assert(ros_interface != nullptr && "RosInterface must not be null");
}

void ros_backend::consume(boost::log::record_view const& rec, 
                          string_type const& formatted) {
    if (!ros_interface_) {
        return;
    }
    
    // Get severity from record
    auto sev = boost::log::extract<severity_level>("Severity", rec);
    if (!sev) {
        // Default to info if no severity
        ros_interface_->log_info(formatted);
        return;
    }
    
    // Forward to appropriate ROS logging function based on severity
    switch (*sev) {
        case severity_level::debug:
            ros_interface_->log_debug(formatted);
            break;
        case severity_level::info:
            ros_interface_->log_info(formatted);
            break;
        case severity_level::warn:
            ros_interface_->log_warn(formatted);
            break;
        case severity_level::error:
        case severity_level::fatal:
            ros_interface_->log_error(formatted);
            break;
        default:
            ros_interface_->log_info(formatted);
            break;
    }
}

boost::shared_ptr<ros_sink_t> create_ros_sink(
    recorder::RosInterface* ros_interface,
    severity_level min_level
) {
    auto backend = boost::make_shared<ros_backend>(ros_interface);
    auto sink = boost::make_shared<ros_sink_t>(backend);
    
    // Set filter
    sink->set_filter(severity >= min_level);
    
    // Simple formatter for ROS - just the message
    // ROS logging already adds timestamp
    sink->set_formatter(
        [](boost::log::record_view const& rec, boost::log::formatting_ostream& strm) {
            // Message (includes component prefix from macros)
            strm << rec[expr::smessage];
            
            // Context (TaskID, DeviceID) - useful for correlation
            auto tid = boost::log::extract<std::string>("TaskID", rec);
            auto did = boost::log::extract<std::string>("DeviceID", rec);
            if (tid || did) {
                strm << " |";
                if (tid) strm << " task_id=" << *tid;
                if (did) strm << " device_id=" << *did;
            }
        }
    );
    
    return sink;
}

}  // namespace logging
}  // namespace axon

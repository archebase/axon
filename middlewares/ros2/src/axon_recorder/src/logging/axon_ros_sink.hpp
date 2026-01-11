#ifndef AXON_ROS_SINK_HPP
#define AXON_ROS_SINK_HPP

#include <axon_log_severity.hpp>

#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "../ros_interface.hpp"

namespace axon {
namespace logging {

/**
 * Custom Boost.Log backend that forwards to ROS logging.
 * Uses synchronous sink (ROS logging is fast, no async needed).
 *
 * This sink bridges Boost.Log records to the native ROS logging system,
 * making logs appear in rqt_console, rosout, etc.
 *
 * IMPORTANT: Lifetime Requirements
 * The RosInterface pointer must remain valid for the entire lifetime of this sink.
 * Typically:
 * 1. Create RosInterface
 * 2. Create and add ROS sink via add_sink()
 * 3. Use logging normally
 * 4. Remove ROS sink via remove_sink() BEFORE destroying RosInterface
 * 5. Destroy RosInterface
 */
class ros_backend : public boost::log::sinks::basic_formatted_sink_backend<
                      char, boost::log::sinks::synchronized_feeding> {
public:
  /**
   * Create ROS backend.
   * @param ros_interface Pointer to ROS interface (must not be null, must outlive this backend)
   * @throws assertion failure if ros_interface is null (in debug builds)
   */
  explicit ros_backend(recorder::RosInterface* ros_interface);

  /**
   * Called by Boost.Log for each record.
   * Forwards the formatted message to the appropriate ROS logging function.
   */
  void consume(boost::log::record_view const& rec, string_type const& formatted);

private:
  recorder::RosInterface* ros_interface_;
};

// Synchronous sink type for ROS (no async needed - ROS logging is fast)
typedef boost::log::sinks::synchronous_sink<ros_backend> ros_sink_t;

/**
 * Create ROS sink that bridges to ROS logging.
 * Logs will appear in rqt_console, rosout, etc.
 *
 * @param ros_interface Pointer to ROS interface (must outlive the sink)
 * @param min_level Minimum severity level to forward to ROS
 * @return Shared pointer to the sink
 */
boost::shared_ptr<ros_sink_t> create_ros_sink(
  recorder::RosInterface* ros_interface, severity_level min_level = severity_level::info
);

}  // namespace logging
}  // namespace axon

#endif  // AXON_ROS_SINK_HPP

// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "axon_console_sink.hpp"

#include <boost/core/null_deleter.hpp>
#include <boost/log/attributes/value_extraction.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/make_shared.hpp>

#include <iostream>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;

// Color codes for terminal output
const char* get_color(severity_level level) {
  switch (level) {
    case severity_level::debug:
      return "\033[36m";  // Cyan
    case severity_level::info:
      return "\033[32m";  // Green
    case severity_level::warn:
      return "\033[33m";  // Yellow
    case severity_level::error:
      return "\033[31m";  // Red
    case severity_level::fatal:
      return "\033[35m";  // Magenta
    default:
      return "";
  }
}

const char* get_reset_color() {
  return "\033[0m";
}

boost::shared_ptr<async_console_sink_t> create_console_sink(
  severity_level min_level, bool use_colors
) {
  // Create backend
  auto backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
  backend->add_stream(boost::shared_ptr<std::ostream>(&std::clog, boost::null_deleter()));
  backend->auto_flush(true);

  // Create async sink with bounded queue
  auto sink = boost::make_shared<async_console_sink_t>(backend);

  // Set filter
  sink->set_filter(severity >= min_level);

  // Set formatter
  if (use_colors) {
    sink->set_formatter(
      [](boost::log::record_view const& rec, boost::log::formatting_ostream& strm) {
        // Get severity
        auto sev_ref = boost::log::extract<severity_level>("Severity", rec);

        // Timestamp
        strm << "[";
        auto time_stamp = boost::log::extract<boost::posix_time::ptime>("TimeStamp", rec);
        if (time_stamp) {
          strm << *time_stamp;
        }
        strm << "] ";

        // Colored severity
        if (sev_ref) {
          strm << get_color(*sev_ref) << "[" << *sev_ref << "]" << get_reset_color() << " ";
        }

        // Message
        strm << rec[expr::smessage];

        // Context (TaskID, DeviceID)
        auto tid = boost::log::extract<std::string>("TaskID", rec);
        auto did = boost::log::extract<std::string>("DeviceID", rec);
        if (tid || did) {
          strm << " |";
          if (tid) strm << " task_id=" << *tid;
          if (did) strm << " device_id=" << *did;
        }
      }
    );
  } else {
    sink->set_formatter(
      [](boost::log::record_view const& rec, boost::log::formatting_ostream& strm) {
        // Timestamp
        strm << "[";
        auto time_stamp = boost::log::extract<boost::posix_time::ptime>("TimeStamp", rec);
        if (time_stamp) {
          strm << *time_stamp;
        }
        strm << "] ";

        // Severity
        auto sev_ref = boost::log::extract<severity_level>("Severity", rec);
        if (sev_ref) {
          strm << "[" << *sev_ref << "] ";
        }

        // Message
        strm << rec[expr::smessage];

        // Context
        auto tid = boost::log::extract<std::string>("TaskID", rec);
        auto did = boost::log::extract<std::string>("DeviceID", rec);
        if (tid || did) {
          strm << " |";
          if (tid) strm << " task_id=" << *tid;
          if (did) strm << " device_id=" << *did;
        }
      }
    );
  }

  return sink;
}

}  // namespace logging
}  // namespace axon

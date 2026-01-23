// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "axon_file_sink.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/attributes/current_thread_id.hpp>
#include <boost/log/attributes/value_extraction.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/make_shared.hpp>

#include <cstdio>
#include <iostream>

namespace axon {
namespace logging {

namespace expr = boost::log::expressions;
namespace keywords = boost::log::keywords;
namespace sinks = boost::log::sinks;

/**
 * Escape a string for JSON output per RFC 8259.
 * All control characters U+0000 through U+001F must be escaped.
 */
std::string escape_json(const std::string& s) {
  std::string result;
  result.reserve(s.size() + 16);
  for (unsigned char c : s) {
    switch (c) {
      case '"':
        result += "\\\"";
        break;
      case '\\':
        result += "\\\\";
        break;
      case '\b':
        result += "\\b";
        break;  // backspace (U+0008)
      case '\f':
        result += "\\f";
        break;  // form feed (U+000C)
      case '\n':
        result += "\\n";
        break;  // newline (U+000A)
      case '\r':
        result += "\\r";
        break;  // carriage return (U+000D)
      case '\t':
        result += "\\t";
        break;  // tab (U+0009)
      default:
        // Escape other control characters (U+0000-U+001F) as \uXXXX
        if (c < 0x20) {
          char buf[8];
          snprintf(buf, sizeof(buf), "\\u%04x", c);
          result += buf;
        } else {
          result += static_cast<char>(c);
        }
    }
  }
  return result;
}

/**
 * JSON formatter for file output.
 */
void json_formatter(boost::log::record_view const& rec, boost::log::formatting_ostream& strm) {
  strm << "{";

  // Timestamp
  strm << "\"ts\":\"";
  auto time_stamp = boost::log::extract<boost::posix_time::ptime>("TimeStamp", rec);
  if (time_stamp) {
    strm << *time_stamp;
  }
  strm << "\",";

  // Severity
  auto sev = boost::log::extract<severity_level>("Severity", rec);
  strm << "\"level\":\"";
  if (sev) {
    strm << *sev;
  }
  strm << "\",";

  // Message (includes component prefix now)
  strm << "\"msg\":\"" << escape_json(rec[expr::smessage].get()) << "\"";

  // Thread ID for debugging multi-threaded issues
  auto thread_id =
    boost::log::extract<boost::log::attributes::current_thread_id::value_type>("ThreadID", rec);
  if (thread_id) {
    strm << ",\"thread_id\":\"" << *thread_id << "\"";
  }

  // Optional context attributes
  auto tid = boost::log::extract<std::string>("TaskID", rec);
  if (tid) {
    strm << ",\"task_id\":\"" << escape_json(*tid) << "\"";
  }

  auto did = boost::log::extract<std::string>("DeviceID", rec);
  if (did) {
    strm << ",\"device_id\":\"" << escape_json(*did) << "\"";
  }

  strm << "}";
}

/**
 * Text formatter for file output.
 */
void text_formatter(boost::log::record_view const& rec, boost::log::formatting_ostream& strm) {
  // Timestamp
  strm << "[";
  auto time_stamp = boost::log::extract<boost::posix_time::ptime>("TimeStamp", rec);
  if (time_stamp) {
    strm << *time_stamp;
  }
  strm << "] ";

  // Severity
  auto sev = boost::log::extract<severity_level>("Severity", rec);
  if (sev) {
    strm << "[" << *sev << "] ";
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

boost::shared_ptr<async_file_sink_t> create_file_sink(
  const FileSinkConfig& config, severity_level min_level
) {
  // Create directory if it doesn't exist
  std::string log_directory = config.directory;
  boost::filesystem::path dir_path(log_directory);

  if (!boost::filesystem::exists(dir_path)) {
    boost::system::error_code ec;
    boost::filesystem::create_directories(dir_path, ec);
    if (ec) {
      // Fall back to /tmp if we can't create the directory
      // Use stderr since logging may not be fully initialized yet
      std::cerr << "[axon_logging] Warning: Could not create log directory '" << config.directory
                << "': " << ec.message() << ". Falling back to /tmp\n";
      log_directory = "/tmp";
    }
  }

  // Build file path pattern
  std::string file_path = log_directory + "/" + config.file_pattern;

  // Create backend with rotation
  auto backend = boost::make_shared<sinks::text_file_backend>(
    keywords::file_name = file_path,
    keywords::rotation_size = config.rotation_size_mb * 1024 * 1024,
    keywords::auto_flush = true
  );

  // Set time-based rotation if enabled
  if (config.rotate_at_midnight) {
    backend->set_time_based_rotation(
      sinks::file::rotation_at_time_point(0, 0, 0)  // Midnight
    );
  }

  // Set up file collector for rotation management
  backend->set_file_collector(
    sinks::file::make_collector(
      keywords::target = log_directory, keywords::max_files = config.max_files
    )
  );

  // Scan for existing files on startup
  backend->scan_for_files();

  // Create async sink
  auto sink = boost::make_shared<async_file_sink_t>(backend);

  // Set filter
  sink->set_filter(severity >= min_level);

  // Set formatter
  if (config.format_json) {
    sink->set_formatter(&json_formatter);
  } else {
    sink->set_formatter(&text_formatter);
  }

  return sink;
}

}  // namespace logging
}  // namespace axon

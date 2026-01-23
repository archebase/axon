// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_CONSOLE_SINK_HPP
#define AXON_CONSOLE_SINK_HPP

#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/bounded_fifo_queue.hpp>
#include <boost/log/sinks/drop_on_overflow.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "axon_log_severity.hpp"

namespace axon {
namespace logging {

/**
 * Async console sink with bounded queue.
 * Drops records on overflow to prevent blocking the hot path.
 */
typedef boost::log::sinks::asynchronous_sink<
  boost::log::sinks::text_ostream_backend,
  boost::log::sinks::bounded_fifo_queue<
    1000,  // Max 1000 records in queue
    boost::log::sinks::drop_on_overflow>>
  async_console_sink_t;

/**
 * Create async console sink with colored output.
 * Uses bounded queue to prevent memory explosion under high load.
 *
 * @param min_level Minimum severity level to log
 * @param use_colors Whether to use ANSI color codes
 * @return Shared pointer to the sink
 */
boost::shared_ptr<async_console_sink_t> create_console_sink(
  severity_level min_level = severity_level::info, bool use_colors = true
);

}  // namespace logging
}  // namespace axon

#endif  // AXON_CONSOLE_SINK_HPP

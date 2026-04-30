// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#ifndef AXON_RECORDER_PERCENTILE_CALCULATOR_HPP
#define AXON_RECORDER_PERCENTILE_CALCULATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace axon {
namespace recorder {

/**
 * PercentileCalculator - Calculates percentiles using t-digest algorithm
 *
 * This class implements a simplified t-digest algorithm for efficient
 * percentile calculation with bounded memory usage. It's suitable for
 * real-time latency monitoring with high throughput.
 *
 * Features:
 * - O(n log n) construction for batch data
 * - O(log n) per-item insertion
 * - O(n) percentile query
 * - Memory-bounded with sampling for large datasets
 */
class PercentileCalculator {
public:
  struct Percentiles {
    uint64_t min_ns = 0;
    uint64_t p50_ns = 0;
    uint64_t p90_ns = 0;
    uint64_t p95_ns = 0;
    uint64_t p99_ns = 0;
    uint64_t p999_ns = 0;
    uint64_t max_ns = 0;
    double mean_ns = 0;
    double std_dev_ns = 0;
    uint64_t count = 0;
  };

  explicit PercentileCalculator(size_t max_samples = 100000);

  void add(uint64_t value_ns);

  void add_batch(const std::vector<uint64_t>& values);

  Percentiles get_percentiles() const;

  void reset();

  size_t size() const;

private:
  struct Centroid {
    double mean;
    double weight;

    Centroid(double m = 0.0, double w = 1.0)
        : mean(m)
        , weight(w) {}
  };

  double get_percentile(double p) const;

  static double normal_compressive_force(double z);

  mutable std::mutex mutex_;
  std::vector<Centroid> centroids_;
  std::vector<uint64_t> raw_samples_;
  size_t max_samples_;
  double sum_ = 0.0;
  double sum_sq_ = 0.0;
};

}  // namespace recorder
}  // namespace axon

#endif  // AXON_RECORDER_PERCENTILE_CALCULATOR_HPP

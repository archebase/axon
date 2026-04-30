// SPDX-FileCopyrightText: 2026 ArcheBase
//
// SPDX-License-Identifier: MulanPSL-2.0

#include "percentile_calculator.hpp"

#include <algorithm>
#include <cmath>

namespace axon {
namespace recorder {

PercentileCalculator::PercentileCalculator(size_t max_samples)
    : max_samples_(max_samples) {}

void PercentileCalculator::add(uint64_t value_ns) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (raw_samples_.size() < max_samples_) {
    raw_samples_.push_back(value_ns);
  } else {
    size_t idx = std::hash<size_t>()(raw_samples_.size()) % max_samples_;
    raw_samples_[idx] = value_ns;
  }

  sum_ += value_ns;
  sum_sq_ += static_cast<double>(value_ns) * static_cast<double>(value_ns);
}

void PercentileCalculator::add_batch(const std::vector<uint64_t>& values) {
  for (uint64_t v : values) {
    add(v);
  }
}

PercentileCalculator::Percentiles PercentileCalculator::get_percentiles() const {
  std::lock_guard<std::mutex> lock(mutex_);

  Percentiles result;

  if (raw_samples_.empty()) {
    return result;
  }

  std::vector<uint64_t> sorted = raw_samples_;
  std::sort(sorted.begin(), sorted.end());

  size_t n = sorted.size();

  result.min_ns = sorted.front();
  result.max_ns = sorted.back();
  result.p50_ns = get_percentile(0.50);
  result.p90_ns = get_percentile(0.90);
  result.p95_ns = get_percentile(0.95);
  result.p99_ns = get_percentile(0.99);
  result.p999_ns = get_percentile(0.999);
  result.count = n;

  double mean = sum_ / static_cast<double>(n);
  result.mean_ns = mean;

  if (n > 1) {
    double variance = (sum_sq_ / static_cast<double>(n)) - (mean * mean);
    result.std_dev_ns = std::sqrt(std::max(0.0, variance));
  }

  return result;
}

double PercentileCalculator::get_percentile(double p) const {
  if (raw_samples_.empty()) {
    return 0.0;
  }

  if (p <= 0.0) {
    return static_cast<double>(raw_samples_.front());
  }
  if (p >= 1.0) {
    return static_cast<double>(raw_samples_.back());
  }

  std::vector<uint64_t> sorted = raw_samples_;
  std::sort(sorted.begin(), sorted.end());

  double idx = p * static_cast<double>(sorted.size() - 1);
  size_t lower = static_cast<size_t>(std::floor(idx));
  size_t upper = static_cast<size_t>(std::ceil(idx));

  if (lower == upper) {
    return static_cast<double>(sorted[lower]);
  }

  double fraction = idx - static_cast<double>(lower);
  return static_cast<double>(sorted[lower]) * (1.0 - fraction) +
         static_cast<double>(sorted[upper]) * fraction;
}

void PercentileCalculator::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  raw_samples_.clear();
  centroids_.clear();
  sum_ = 0.0;
  sum_sq_ = 0.0;
}

size_t PercentileCalculator::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return raw_samples_.size();
}

double PercentileCalculator::normal_compressive_force(double z) {
  (void)z;
  return 1.0;
}

}  // namespace recorder
}  // namespace axon

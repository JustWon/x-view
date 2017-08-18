#include <x_view_evaluation/statistics.h>

namespace x_view_evaluation {

Statistics::Statistics()
    : sum_(0.0),
      num_samples_(0) {
}

void Statistics::insert(const x_view::real_t& sample) {
  ++num_samples_;
  sum_ += sample;
}

const x_view::real_t Statistics::mean() const {
  return sum_ / num_samples_;
}

const uint64_t Statistics::numSamples() const {
  return num_samples_;
}

}

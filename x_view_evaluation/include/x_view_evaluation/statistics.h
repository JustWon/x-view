#ifndef X_VIEW_EVALUATION_STATISTICS_H
#define X_VIEW_EVALUATION_STATISTICS_H

#include <x_view_core/x_view_types.h>

namespace x_view_evaluation {

class Statistics {
 public:
  Statistics();

  void insert(const x_view::real_t& sample);
  const x_view::real_t mean() const;
  const uint64_t numSamples() const;

 private:
  x_view::real_t sum_;
  uint64_t num_samples_;
};

}
#endif //X_VIEW_EVALUATION_STATISTICS_H

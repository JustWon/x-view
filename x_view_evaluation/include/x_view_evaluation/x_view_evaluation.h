#ifndef X_VIEW_EVALUATION_H
#define X_VIEW_EVALUATION_H

#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

namespace x_view_evaluation {

struct EvaluationParameters {

  enum TIMER_TYPE {
    NULL_TIMER = 0,
    TIMER
  };

  EvaluationParameters();

  TIMER_TYPE timer_type;

};

class Evaluation {
 public:

  typedef EvaluationParameters::TIMER_TYPE TIMER_TYPE;

  Evaluation(const EvaluationParameters& params);

  const std::string getTimingsTable() const;
  const std::unordered_map<std::string, std::vector<x_view::real_t>>
      getAllTimings() const;
 private:
  static void initializeTimer(const TIMER_TYPE timer_type);
  const EvaluationParameters params_;
};

}

#endif //X_VIEW_EVALUATION_H

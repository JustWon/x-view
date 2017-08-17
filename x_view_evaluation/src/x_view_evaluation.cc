#include <x_view_evaluation/x_view_evaluation.h>

#include <x_view_core/timer/timer_printer.h>
#include <x_view_core/x_view_locator.h>

namespace x_view_evaluation {

EvaluationParameters::EvaluationParameters()
    : timer_type(TIMER_TYPE::NULL_TIMER) {
}

Evaluation::Evaluation(const EvaluationParameters& params)
: params_(params) {

  Evaluation::initializeTimer(params_.timer_type);
}

void Evaluation::initializeTimer(const TIMER_TYPE timer_type) {

  switch(timer_type) {
    case TIMER_TYPE::NULL_TIMER: {
      std::unique_ptr<x_view::AbstractTimer> timer(new x_view::NullTimer());
      x_view::Locator::registerTimer(std::move(timer));
      return;
    }
    case TIMER_TYPE::TIMER: {
      std::unique_ptr<x_view::AbstractTimer> timer(new x_view::Timer());
      x_view::Locator::registerTimer(std::move(timer));
      return;
    }
    default:
      LOG(ERROR) << "Undefined timer type passed to " << __FUNCTION__ << ".";
  }
}

const std::string Evaluation::getTimingsTable() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_.timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
        << "Function " << __FUNCTION__ << " is only supported for timers of "
            "type <x_view::Timer>";

  return x_view::TimerPrinter::getTimingsTable(*real_timer);
}

const std::string Evaluation::getTimingsTree() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_.timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return "Registered timer is of <nonMeasuring> type, this means that the "
        "registered instance does not measure any time.";
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
  << "Function " << __FUNCTION__ << " is only supported for timers of "
      "type <x_view::Timer>";

  return x_view::TimerPrinter::getTimingsTree(*real_timer);
}

const std::unordered_map<std::string, std::vector<x_view::real_t>>
Evaluation::getAllTimings() const {
  const auto& timer = x_view::Locator::getTimer();
  if(params_.timer_type == EvaluationParameters::TIMER_TYPE::NULL_TIMER) {
    return {{"No measurements where made", {}}};
  }

  x_view::Timer* real_timer = dynamic_cast<x_view::Timer*>(timer.get());
  CHECK(real_timer != nullptr)
  << "Function " << __FUNCTION__ << " is only supported for timers of "
      "type <x_view::Timer>";

  return real_timer->getAllTimings();
};

void Evaluation::storeTimer(x_view::AbstractTimer** timer) const {
  *timer = x_view::Locator::removeTimer();

  initializeTimer(params_.timer_type);
}

}


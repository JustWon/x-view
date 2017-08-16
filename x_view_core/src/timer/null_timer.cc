#include <x_view_core/timer/null_timer.h>

namespace x_view {

NullTimer::NullTimer()
 : AbstractTimer() {

}

bool NullTimer::registerTimer(const std::string& timer_name) {
  return false;
}

void NullTimer::start(const std::string& timer_name) {
}

const AbstractTimer::ElapsedTimeType NullTimer::stop(
    const std::string& timer_name)  {
  return AbstractTimer::ElapsedTimeType::zero();
};

}


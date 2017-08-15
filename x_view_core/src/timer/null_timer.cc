#include <x_view_core/timer/null_timer.h>

namespace x_view {

NullTimer::NullTimer()
 : AbstractTimer() {

}

bool NullTimer::registerTimer(const std::string& timer_name) {
}

void NullTimer::start(const std::string& timer_name) {
}

const std::chrono::duration<real_t, std::ratio<1, 1>> NullTimer::stop(const std::string& timer_name)  {
  return std::chrono::duration<real_t, std::ratio<1, 1> >::zero();
};

}


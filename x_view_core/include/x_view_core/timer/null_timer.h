#ifndef X_VIEW_NULL_TIMER_H
#define X_VIEW_NULL_TIMER_H

#include <x_view_core/timer/abstract_timer.h>
#include <x_view_core/x_view_types.h>

#include <chrono>

namespace x_view {

class NullTimer : public AbstractTimer {

 public:
  NullTimer();

  virtual bool registerTimer(const std::string& timer_name) override;
  virtual void start(const std::string& timer_name) override;
  virtual const std::chrono::duration<real_t, std::ratio<1, 1>> stop(const std::string& timer_name) override;

};


}

#endif //X_VIEW_TIMER_H

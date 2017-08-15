#ifndef X_VIEW_ABSTRACT_TIMER_H
#define X_VIEW_ABSTRACT_TIMER_H

#include <x_view_core/x_view_types.h>

#include <chrono>
#include <string>

namespace x_view {

class AbstractTimer {

 public:

  virtual bool registerTimer(const std::string& timer_name) = 0;
  virtual void start(const std::string& timer_name) = 0;
  virtual void stop(const std::string& timer_name) = 0;
  virtual const std::chrono::duration<real_t, std::ratio<1, 1>> elapsedTime(
      const std::string& timer_name) = 0;
};


}

#endif //X_VIEW_ABSTRACT_TIMER_H

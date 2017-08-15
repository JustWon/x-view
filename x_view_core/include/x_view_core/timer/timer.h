#ifndef X_VIEW_TIMER_H
#define X_VIEW_TIMER_H

#include <x_view_core/timer/abstract_timer.h>
#include <x_view_core/x_view_types.h>

#include <unordered_map>

namespace x_view {

class Timer : public AbstractTimer {

 public:
  Timer();

  virtual bool registerTimer(const std::string& timer_name) override;
  virtual void start(const std::string& timer_name) override;
  virtual const std::chrono::duration<real_t, std::ratio<1, 1>> stop(const std::string& timer_name) override;

 private:
  class TimerNode {
   public:
    TimerNode();
    void start();
    const std::chrono::steady_clock::duration stop();

   private:

    bool has_been_started_;
    bool has_been_stopped_;

    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::duration elapsed_time_;
  };

  std::unordered_map<std::string, TimerNode> timers_;
};


}

#endif //X_VIEW_TIMER_H

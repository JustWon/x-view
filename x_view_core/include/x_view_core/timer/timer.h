#ifndef X_VIEW_TIMER_H
#define X_VIEW_TIMER_H

#include <glog/logging.h>

#include <chrono>
#include <unordered_map>

namespace x_view {

class TimeManager {

 private:
  class Timer {
   public:
    Timer();
    void start();
    void stop();
    const std::chrono::steady_clock::duration elapsedTime();

   private:

    bool has_been_started_;
    bool has_been_stopped_;

    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::duration elapsed_time_;
  };

 public:
  TimeManager();

  bool registerTimer(const std::string& timer_name);
  void start(const std::string& timer_name);
  void stop(const std::string& timer_name);
  const std::chrono::steady_clock::duration elapsedTime(
      const std::string& timer_name);

 private:
  std::unordered_map<std::string, Timer> timers_;
};


}

#endif //X_VIEW_TIMER_H

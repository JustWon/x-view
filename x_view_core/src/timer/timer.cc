#include <x_view_core/timer/timer.h>

namespace x_view {

TimeManager::Timer::Timer()
    : has_been_started_(false),
      has_been_stopped_(false) {

}

void TimeManager::Timer::start() {
  CHECK(has_been_started_ == false)
        << "Starting timer twice without stopping it first.";
  has_been_started_ = true;
  start_ = std::chrono::steady_clock::now();
}

void TimeManager::Timer::stop() {
  auto end_time = std::chrono::steady_clock::now();
  CHECK(has_been_started_ == true)
        << "Stopping timer without starting it first.";
  CHECK(has_been_stopped_ == false)
        << "Stopping timer twice.";
  has_been_stopped_ = true;
  elapsed_time_ = (end_time - start_);
}

const std::chrono::steady_clock::duration TimeManager::Timer::elapsedTime() {
  has_been_started_ = false;
  has_been_stopped_ = false;

  auto return_elapsed_time = elapsed_time_;
  elapsed_time_ = std::chrono::steady_clock::duration(0);

  return return_elapsed_time;
}

TimeManager::TimeManager() {
}

bool TimeManager::registerTimer(const std::string& timer_name) {
  if(timers_.count(timer_name) > 0)
    return false;
  timers_.insert({timer_name, Timer()});
  return true;
}

void TimeManager::start(const std::string& timer_name) {
  CHECK(timers_.count(timer_name) > 0)
        << "Requested timer <" << timer_name << "> without registering it "
            "first.";
  timers_[timer_name].start();
}

void TimeManager::stop(const std::string& timer_name) {
  CHECK(timers_.count(timer_name) > 0)
  << "Requested timer <" << timer_name << "> without registering it "
      "first.";
  timers_[timer_name].stop();
}

const std::chrono::duration<real_t, std::ratio<1, 1>> TimeManager::elapsedTime(
    const std::string& timer_name) {
  CHECK(timers_.count(timer_name) > 0)
  << "Requested timer <" << timer_name << "> without registering it "
      "first.";
  return timers_.at(timer_name).elapsedTime();
}

}


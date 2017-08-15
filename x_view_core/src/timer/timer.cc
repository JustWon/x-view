#include <x_view_core/timer/timer.h>

#include <glog/logging.h>

namespace x_view {

Timer::TimerNode::TimerNode()
    : has_been_started_(false),
      has_been_stopped_(false) {

}

void Timer::TimerNode::start() {
  CHECK(has_been_started_ == false)
        << "Starting timer twice without stopping it first.";
  has_been_started_ = true;
  has_been_stopped_ = false;
  start_ = std::chrono::steady_clock::now();
}

const std::chrono::steady_clock::duration Timer::TimerNode::stop() {
  const auto end_time = std::chrono::steady_clock::now();
  if(!has_been_stopped_) {
	  CHECK(has_been_started_ == true)
        	<< "Stopping timer without starting it first.";
  	  has_been_stopped_ = true;
  	  elapsed_time_ = (end_time - start_);
  }

  has_been_started_ = false;
  return elapsed_time_;
}

Timer::Timer()
    : AbstractTimer() {
}

bool Timer::registerTimer(const std::string& timer_name) {
  if(timers_.count(timer_name) > 0)
    return false;
  timers_.insert({timer_name, TimerNode()});
  return true;
}

void Timer::start(const std::string& timer_name) {
  CHECK(timers_.count(timer_name) > 0)
        << "Requested timer <" << timer_name << "> without registering it "
            "first.";
  timers_[timer_name].start();
}

const std::chrono::duration<real_t, std::ratio<1, 1>> Timer::stop(const std::string& timer_name) {
  CHECK(timers_.count(timer_name) > 0)
  << "Requested timer <" << timer_name << "> without registering it "
      "first.";
  return timers_[timer_name].stop();
}
}


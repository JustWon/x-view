#include <x_view_core/timer/timer.h>

#include <glog/logging.h>

#include <iomanip>
#include <sstream>

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
  if (!has_been_stopped_) {
    CHECK(has_been_started_ == true)
    << "Stopping timer without starting it first.";
    has_been_stopped_ = true;
    elapsed_time_ = (end_time - start_);
  }

  has_been_started_ = false;
  return elapsed_time_;
}

const std::chrono::steady_clock::duration Timer::TimerNode::elapsedTime() const {
  CHECK(has_been_stopped_)
  << "Requesting elapsed time of TimerNode without stopping it.";
  return elapsed_time_;
}

Timer::Timer()
    : AbstractTimer() {
}

bool Timer::registerTimer(const std::string& timer_name) {
  if (timer_map_.count(timer_name) > 0)
    return false;
  timer_map_.insert({timer_name, {}});
  return true;
}

void Timer::start(const std::string& timer_name) {
  CHECK(timer_map_.count(timer_name) > 0)
  << "Requested timer <" << timer_name << "> without registering it first.";
  timer_map_[timer_name].push_back(TimerNode());
  timer_map_[timer_name].back().start();
}

const AbstractTimer::ElapsedTimeType Timer::stop(
    const std::string& timer_name) {
  CHECK(timer_map_.count(timer_name) > 0)
  << "Requested timer <" << timer_name << "> without registering it first.";
  return timer_map_[timer_name].back().stop();
}

const std::string Timer::getTimingsTable() const {

  const uint64_t function_name_width = 25;
  const uint64_t col_width = 9;
  const std::string col_sep = " | ";
  std::stringstream ss;

  auto getRightString = [&](const std::string& s) -> std::string {
    const uint64_t string_length = s.length();
    if (string_length > function_name_width)
      return s.substr(0, function_name_width - 1) + ".";

    const uint64_t remaining_space = function_name_width - string_length;
    std::string center_string(remaining_space, ' ');
    return center_string + s;
  };

  auto getLeftString = [&](const std::string& s) -> std::string {
    const uint64_t string_length = s.length();
    if (string_length > col_width)
      return s.substr(0, col_width - 1) + ".";

    const uint64_t remaining_space = col_width - string_length;
    std::string center_string(remaining_space, ' ');
    return s + center_string;
  };

  ss << getRightString("Timer") << col_sep;
  ss << getLeftString("mean [s]") << col_sep;
  ss << getLeftString("std [s]") << col_sep;
  ss << getLeftString("num");
  ss << "\n";
  const uint64_t line_width = ss.str().length();
  ss << std::setfill('=') << std::setw(line_width);
  ss << "\n";

  for (const auto& p : timer_map_) {
    ss << getRightString(p.first) << col_sep;
    ss << getLeftString(std::to_string(getMean(p.second))) << col_sep;
    ss << getLeftString(std::to_string(getStd(p.second))) << col_sep;
    ss << getLeftString(std::to_string(p.second.size()));
    ss << "\n";
  }

  return ss.str();
}

const std::vector<x_view::real_t> Timer::getTimes(
    const std::string& timer_name) const {
  CHECK(timer_map_.count(timer_name) > 0)
        << "Requesting times of unregistered timer <" << timer_name << ">.";

  std::vector<x_view::real_t> times;
  const std::vector<TimerNode>& timings = timer_map_.at(timer_name);
  for(const TimerNode& t : timings) {
    std::chrono::duration<x_view::real_t, std::ratio<1, 1>> seconds =
        t.elapsedTime();
    times.push_back(seconds.count());
  }
  return times;
}

const std::unordered_map<std::string, std::vector<x_view::real_t>>
Timer::getAllTimings() const {
  std::unordered_map<std::string, std::vector<x_view::real_t>> all_times;
  for(const auto& p : timer_map_) {
    const std::string& timer_name = p.first;
    const std::vector<x_view::real_t> times = getTimes(timer_name);
    all_times.insert({timer_name, times});
  }
  return all_times;
}

x_view::real_t Timer::getMean(const std::vector<TimerNode>& timers) {
  std::chrono::steady_clock::duration sum =
      std::chrono::steady_clock::duration::zero();
  for (const TimerNode& t : timers) {
    sum += t.elapsedTime();
  }
  sum /= timers.size();
  std::chrono::duration<x_view::real_t, std::ratio<1, 1>> mean_seconds = sum;
  return mean_seconds.count();
}

x_view::real_t Timer::getStd(const std::vector<TimerNode>& timers) {
  if(timers.size() < 2)
    return static_cast<x_view::real_t>(0.0);

  const x_view::real_t mean = getMean(timers);
  x_view::real_t std = static_cast<x_view::real_t>(0.0);
  for (const TimerNode& t : timers) {
    std::chrono::duration<x_view::real_t, std::ratio<1, 1>> duration_sec =
        t.elapsedTime();
    std += (duration_sec.count() - mean) * (duration_sec.count() - mean);
  }

  return std::sqrt(1.0 / (timers.size() - 1) * std);
}
}


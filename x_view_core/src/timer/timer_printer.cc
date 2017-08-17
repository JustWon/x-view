#include <x_view_core/timer/timer_printer.h>

#include <iomanip>
#include <sstream>

namespace x_view {

const std::string TimerPrinter::TIMER_INDENTATION = "|  ";


const std::string TimerPrinter::getTimingsTable(const Timer& timer) {

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

  for(const std::string s : timer.insertion_order_) {
    const std::vector<Timer::TimerNode>& timings = timer.timer_map_.at(s);
    ss << getRightString(s) << col_sep;
    ss << getLeftString(std::to_string(Timer::getMean(timings))) << col_sep;
    ss << getLeftString(std::to_string(Timer::getStd(timings))) << col_sep;
    ss << getLeftString(std::to_string(timings.size()));
    ss << "\n";
  }

  return ss.str();
}

const std::string TimerPrinter::getTimingsTree(const Timer& timer) {
  std::string s;
  // Generate a list of TimeNode which have no parents (roots)
  const std::set<std::string>& root_timers = timer.children_timers_.at("root");
  const std::string& root_indentation = TIMER_INDENTATION;

  for(const std::string& root_timer : root_timers) {
    s += boldify(color(root_timer, COLOR::GREEN)) + ": ";
    s += color("mean: ", COLOR::RED);
    s += std::to_string(Timer::getMean(timer.timer_map_.at(root_timer))) + "s, ";
    s += color("std: ", COLOR::RED);
    s += std::to_string(Timer::getStd(timer.timer_map_.at(root_timer))) + "s ,";
    s += color("#: ", COLOR::RED);
    s += std::to_string(timer.timer_map_.at(root_timer).size());

    s += getSubTreeTimings(timer, root_timer, root_indentation);

    s += "\n";
  }
  return s;
}

const std::string TimerPrinter::getSubTreeTimings(
    const Timer& timer, const std::string& parent_timer,
    const std::string& indentation) {

  if(timer.children_timers_.count(parent_timer) > 0) {
    const std::set<std::string>& children_timers =
        timer.children_timers_.at(parent_timer);
    const x_view::real_t parent_mean =
        Timer::getMean(timer.timer_map_.at(parent_timer));
    std::string s;
    for (const std::string& children_timer : children_timers) {
      s += "\n" + indentation + "|\n" + indentation + "+--";
      const x_view::real_t children_mean =
          Timer::getMean(timer.timer_map_.at(children_timer));
      const x_view::real_t percent = (100 * children_mean) / parent_mean;
      std::stringstream percent_stream;
      percent_stream << std::fixed << std::setprecision(2) << percent;

      s += boldify("[" + percent_stream.str() + "%] ");
      s += boldify(color(children_timer, COLOR::GREEN)) + ": ";
      s += color("mean: ", COLOR::RED) +
          std::to_string(Timer::getMean(timer.timer_map_.at(children_timer))) +
          "s, ";
      s += color("std: ", COLOR::RED) +
          std::to_string(Timer::getStd(timer.timer_map_.at(children_timer))) +
          "s, ";
      s += color("#: ", COLOR::RED) +
          std::to_string(timer.timer_map_.at(children_timer).size());

      s += getSubTreeTimings(timer, children_timer, indentation +
          TIMER_INDENTATION);
    }
    s+= "\n" + indentation;
    return s;
  }
  return "";
}

const std::string TimerPrinter::boldify(const std::string& s) {
  return "\e[1m" + s + "\e[0m";
}

const std::string TimerPrinter::color(const std::string& s,
                                      const COLOR color) {
  return "\033[" + std::to_string(color) + "m" + s +
      "\033[" + std::to_string(COLOR::DEF) + "m";
}

}

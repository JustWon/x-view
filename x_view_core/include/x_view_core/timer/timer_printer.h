#ifndef X_VIEW_TIMER_PRINTER_H
#define X_VIEW_TIMER_PRINTER_H

#include <x_view_core/timer/timer.h>

namespace x_view {

class TimerPrinter {

 public:
  /**
 * \brief Generates a human readable table with information about the
 * registered timers.
 * \return A string containing the generated table.
 */
  static const std::string getTimingsTable(const Timer& timer);

  static const std::string getTimingsTree(const Timer& timer);

 private:
  static const std::string getSubTreeTimings(const Timer& timer,
                                             const std::string& parent_timer,
                                             const std::string& indentation);

  enum COLOR {
    RED = 31,
    GREEN = 32,
    BLUE = 34,
    DEF = 39
  };

  static const std::string boldify(const std::string& s);
  static const std::string color(const std::string& s, const COLOR color);

  static const std::string TIMER_INDENTATION;
};

}

#endif //X_VIEW_TIMER_PRINTER_H

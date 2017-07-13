#ifndef X_VIEW_PAUSE_H
#define X_VIEW_PAUSE_H

#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

namespace x_view_ros {

/**
 * \brief Class used to pause a for/while loop until a key is pressed.
 */
class Pause {
 public:
  /**
   * \brief Constructor of class instance.
   * \param c Character to wait for (used to (un)trigger pause).
   * \param wait Time span to be waited between two checks of the pause state
   * of the current instance, if the first check reported a paused state.
   */
  Pause(const char c = ' ',
        const std::chrono::milliseconds& wait = std::chrono::milliseconds(200));

  bool isPaused() const;
  void terminate();

 private:
  bool paused_;
  bool is_running_;
  const std::chrono::milliseconds wait_;

  std::thread key_listener;

  void keyListener(const char c);
  void disableEnter() const;
};
}

#endif //X_VIEW_PAUSE_H

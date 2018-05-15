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

  /**
   * \brief Activates or deactivates the Pause behaviour.
   * \param active Boolean flag, if set to true, the Pause behaviour of new
   * Pause instances is valid, otherwise the instances do nothing.
   * \note This function can be called to deactivate/activate the Pause
   * instances globally without needing to remove them from the code.
   * \code{cpp}
   * Pause::activate(false);
   * Pause firstPause;
   * for(int i = 0; ..)
   * { // Press space bar won't affect the code, as the pause is disabled. }
   * firstPause.terminate()
   *
   * Pause::activate(true);
   * Pause secondPause;
   * for(int i = 0; ..)
   * { // Press space bar will affect the code, as the pause is enabled. }
   * secondPause.terminate()
   * \endcode
   */
  static void activate(const bool active = true);

  bool isPaused() const;
  void terminate();

 private:
  bool paused_;
  bool is_running_;
  const std::chrono::milliseconds wait_;

  std::thread key_listener;

  void keyListener(const char c);
  void disableEnter() const;

  /// \brief Flag which can be disabled to avoid the pause to work.
  static bool ACTIVE;

};
}

#endif //X_VIEW_PAUSE_H

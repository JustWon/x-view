#include <x_view_bag_reader/x_view_pause.h>

#include <iostream>

namespace x_view_ros {

Pause::Pause(const char c, const std::chrono::milliseconds& wait)
    : paused_(false),
      is_running_(true),
      wait_(wait),
      key_listener(&Pause::keyListener, this, c) {
}

bool Pause::isPaused() const {
  if (paused_)
    std::this_thread::sleep_for(wait_);
  return paused_;
}

void Pause::terminate()  {
  std::cout << "Press any key to continue." << std::endl;
  is_running_ = false;
  key_listener.join();
}

void Pause::keyListener(const char c) {
  disableEnter();
  char input;
  while(is_running_) {
    std::cin >> std::noskipws >> input;
    if(input == c) {
      paused_ = !paused_;
    }
  }
}

void Pause::disableEnter() const {
  struct termios t;
  // Get the current terminal I/O structure.
  tcgetattr(STDIN_FILENO, &t);
  // Avoid being forced to press 'enter' for using 'cin'.
  t.c_lflag &= ~ICANON;
  // Apply the new settings;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

}

#include "test_timer.h"

#include <chrono>
#include <thread>

namespace x_view_test {

void waitOneSecond() {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return;
}


void waitTwoSeconds() {
  std::this_thread::sleep_for(std::chrono::seconds(2));
  return;
}

}

#include <gtest/gtest.h>
#include "test_timer.h"

#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

#include <glog/logging.h>

#include <chrono>

using namespace x_view;
using namespace x_view_test;


TEST(XViewSlamTestSuite, test_timer) {

  LOG(INFO) << "\n\n====Testing timer====";

  // Define a duration to be slept by inside the called functions.
  std::chrono::duration<x_view::real_t> sleep_duration(static_cast<x_view::real_t>(0.05));

  // Check that the real timer implementation works as expected,
  // i.e. by measuring the elapsed time precisely.
  Timer timer;
  timer.registerTimer("Function1");
  timer.registerTimer("Function2");

  timer.start("Function1");
  waitFunction(sleep_duration);
  timer.stop("Function1");
  CHECK_NEAR(timer.elapsedTime("Function1").count(), sleep_duration.count(), static_cast<x_view::real_t>(0.001));

  timer.start("Function2");
  waitFunction(sleep_duration * 2);
  timer.stop("Function2");
  CHECK_NEAR(timer.elapsedTime("Function2").count(), sleep_duration.count() * 2, static_cast<x_view::real_t>(0.001));


  // Check that the NullTimer does not measure any time duration.
  NullTimer null_timer;
  null_timer.registerTimer("Function1");
  null_timer.registerTimer("Function2");

  null_timer.start("Function1");
  waitFunction(sleep_duration);
  null_timer.stop("Function1");

  CHECK_NEAR(null_timer.elapsedTime("Function1").count(), static_cast<x_view::real_t>(0.0), static_cast<x_view::real_t>(0.001));

}



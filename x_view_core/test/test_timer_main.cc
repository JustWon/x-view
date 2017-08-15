#include <gtest/gtest.h>
#include "test_timer.h"

#include <x_view_core/timer/null_timer.h>
#include <x_view_core/timer/timer.h>

#include <glog/logging.h>

using namespace x_view;
using namespace x_view_test;


TEST(XViewSlamTestSuite, test_timer) {

  LOG(INFO) << "\n\n====Testing timer====";

  Timer timer;
  timer.registerTimer("Function1");
  timer.registerTimer("Function2");

  timer.start("Function1");
  waitOneSecond();
  timer.stop("Function1");
  std::cout << "Elapsed time: "
            << timer.elapsedTime("Function1").count() << " s." << std::endl;


  timer.start("Function2");
  waitTwoSeconds();
  timer.stop("Function2");
  std::cout << "Elapsed time: "
            << timer.elapsedTime("Function2").count() << " s." << std::endl;

  NullTimer null_timer;
  null_timer.registerTimer("Function1");
  null_timer.registerTimer("Function2");

  null_timer.start("Function1");
  waitOneSecond();
  null_timer.stop("Function1");
  std::cout << "Elapsed time: "
            << null_timer.elapsedTime("Function1").count() << " s." << std::endl;


  null_timer.start("Function2");
  waitTwoSeconds();
  null_timer.stop("Function2");
  std::cout << "Elapsed time: "
            << null_timer.elapsedTime("Function2").count() << " s." << std::endl;
}



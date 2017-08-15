#include <gtest/gtest.h>
#include "test_timer.h"

#include <x_view_core/timer/timer.h>

using namespace x_view;
using namespace x_view_test;


TEST(XViewSlamTestSuite, test_timer) {

  LOG(INFO) << "\n\n====Testing timer====";

  TimeManager time_manager;
  time_manager.registerTimer("Function1");
  time_manager.registerTimer("Function2");

  time_manager.start("Function1");
  waitOneSecond();
  time_manager.stop("Function1");
  std::cout << "Elapsed time: "
     << time_manager.elapsedTime("Function1").count() << " s." << std::endl;


  time_manager.start("Function2");
  waitTwoSeconds();
  time_manager.stop("Function2");
  std::cout << "Elapsed time: "
     << time_manager.elapsedTime("Function2").count() << " s." << std::endl;
}



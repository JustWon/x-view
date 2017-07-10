#include <gtest/gtest.h>

#include "test_localization.h"

#include <glog/logging.h>

TEST(XViewSlamTestSuite, test_localization) {

  LOG(INFO) << "\n\n====Testing Localization====";

  const int num_tests = 50;
  const int num_observations = 10;
  const int seed = 0;
  x_view_test::testLocalization(num_tests, num_observations, seed);

}




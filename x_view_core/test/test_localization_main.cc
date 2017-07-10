#include <gtest/gtest.h>

#include "test_localization.h"

#include <glog/logging.h>

TEST(XViewSlamTestSuite, test_localization) {

  LOG(INFO) << "\n\n====Testing Localization====";

  const int num_tests = 1000;
  const int seed = 0;
  const int num_observations = 12;
  x_view_test::testLocalization(num_tests, num_observations, seed);

  x_view_test::testLocalizationUnderRotation();
}




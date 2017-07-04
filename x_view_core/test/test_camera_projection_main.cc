#include <gtest/gtest.h>

#include "test_camera_projection.h"

#include <glog/logging.h>

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_camera_projection) {

  LOG(INFO) << "\n\n====Testing camera projection====";

  testTransforms();
  testPixelToCamera();
  testRandomCameraPose();

}



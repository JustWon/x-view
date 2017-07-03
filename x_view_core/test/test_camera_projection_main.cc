#include <gtest/gtest.h>

#include <x_view_core/landmarks/graph_landmark/projector.h>

#include <random>

using namespace x_view;

TEST(XViewSlamTestSuite, test_camera_projection) {

  LOG(INFO) << "\n\n====Testing camera projection====";

  // Random number generator
  const unsigned long seed = 0;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> distribution(-5, 5);

  // Static world object coordinates.
  Eigen::Vector3d world_coords(0, 0, 15);
  CameraIntrinsics camera_intrinsics(500, 400, 300, 2);

  const int num_reps = 1000;
  for(int i = 0; i < num_reps; ++i) {
    SE3 se3;
    se3.setIdentity();
    se3.getRotationMatrix() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    se3.getPosition()
        << distribution(rng), distribution(rng), distribution(rng);

    const double depth = (world_coords - se3.getPosition()).norm();

    Projector projector(se3, camera_intrinsics);

    const cv::Point2i pixel_coord = projector.getPixelCoordinates(world_coords);

    const Eigen::Vector3d new_world_coord =
        projector.getWorldCoordinates(pixel_coord, depth);

    for(int k = 0; k < 3; ++k)
      CHECK_NEAR(world_coords[k], new_world_coord[k], 0.05);

  }

}



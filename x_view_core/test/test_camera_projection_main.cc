#include <gtest/gtest.h>

#include <x_view_core/landmarks/graph_landmark/projector.h>

using namespace x_view;

TEST(XViewSlamTestSuite, test_camera_projection) {

  LOG(INFO) << "\n\n====Testing camera projection====";

  /// Static world object coordinates.
  Eigen::Vector3d world_coords(10, 10, 20);
  CameraIntrinsics camera_intrinsics(500, 400, 300, 2);

  for(double x = 0.0; x < 5.0; x += 1.0)
  for(double y = 0.0; y < 5.0; y += 1.0) {

    SE3 se3;
    se3.setIdentity();
    se3.getPosition() = Eigen::Vector3d(x, y, 0);

    Projector projector(se3, camera_intrinsics);
    const double dist = (world_coords - se3.getPosition()).norm();

    Eigen::Vector2i pixel_coords = projector.projectWorldToPixel(world_coords);
    std::cout << "Pixel coordinates of " << world_coords << " seen from "
    << se3 << " are " << pixel_coords << std::endl;

    Eigen::Vector3d back_projected =
        projector.projectPixelToWorld(pixel_coords, dist);

    std::cout << "Backprojected is: " << back_projected << std::endl;

  }

}



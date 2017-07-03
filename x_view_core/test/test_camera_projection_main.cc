#include <gtest/gtest.h>

#include <x_view_core/landmarks/graph_landmark/projector.h>

using namespace x_view;

TEST(XViewSlamTestSuite, test_camera_projection) {

  LOG(INFO) << "\n\n====Testing camera projection====";

  /// Static world object coordinates.
  Eigen::Vector3d world_coords(0, 0, 20);
  CameraIntrinsics camera_intrinsics(500, 400, 300, 2);

  for(double z = 0; z < 20; z += 1.0) {
    SE3 se3;
    se3.setIdentity();
    se3.getRotationMatrix() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    se3.getPosition() = Eigen::Vector3d(5, 0, z);

    const double depth = (world_coords - se3.getPosition()).norm();
    std::cout << "Depth is: " << depth << std::endl;

    Projector projector(se3, camera_intrinsics);

    const Eigen::Vector3d camera_coords =
        projector.projectWorldToCamera(world_coords);

    const Eigen::Vector2i pixel_coords =
        projector.projectCameraToPixel(camera_coords);

    const Eigen::Vector3d new_camera_coord =
        projector.projectPixelToCamera(pixel_coords);

    const Eigen::Vector3d new_world_coord =
        projector.projectCameraToWorld(new_camera_coord);

    const Eigen::Vector3d reprojected = se3.getPosition() + new_world_coord *
        depth;

    std::cout << "Recomputed pos: " << reprojected << std::endl;

  }

}



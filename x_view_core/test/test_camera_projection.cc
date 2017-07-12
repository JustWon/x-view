#include "test_camera_projection.h"

#include <x_view_core/landmarks/graph_landmark/depth_projector.h>

#include <glog/logging.h>

#include <random>

namespace x_view_test {

void transformsExample() {

  // Position of robot expressed in world coordinates.
  Eigen::Vector3d robot_position_in_world_frame;
  robot_position_in_world_frame << 10, 50, 200;

  Eigen::Matrix3d rotation;
  rotation << 0, 0, 1,
              1, 0, 0,
              0, 1, 0;

  Eigen::Quaterniond q(rotation);
  x_view::SE3 pose(q, robot_position_in_world_frame);

  std::cout << "Pose:\n" << pose << std::endl;
  std::vector<Eigen::Vector3d> positions {
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1}
  };
  for(const auto& v : positions) {
    std::cout << Eigen::RowVector3d(v) << " in robot frame expressed in "
        "world's frame: " << Eigen::RowVector3d(pose.transform(v))
              << std::endl;
    std::cout << Eigen::RowVector3d(v) << " in world frame expressed in "
        "robot's frame: " << Eigen::RowVector3d(pose.inverseTransform(v)) << std::endl;
    std::cout << std::endl;
  }
}

void testRandomCameraPose() {
  const double tol = 0.1;

  // Random number generator
  const unsigned long seed = 0;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> pos_rand(-5, 5);
  std::uniform_real_distribution<double> angle_rand(0.0, 1.0);

  // Static world object coordinates.
  Eigen::Vector3d object_in_world_coordinates(0, 0, 15);

  const int image_width = 1240;
  const int image_height = 760;
  Eigen::Vector2i principal_point;
  principal_point << image_width / 2, image_height / 2;
  const double focal_length = 400;
  x_view::CameraIntrinsics intrinsics(focal_length, principal_point);

  const int num_reps = 1000;
  for (int i = 0; i < num_reps; ++i) {
    // Random robot position.
    Eigen::Vector3d robot_position;
    robot_position << pos_rand(rng), pos_rand(rng), pos_rand(rng);

    // Random robot orientation with pure random rotation around Z axis
    // (depth direction of camera), and small rotation around other axis to
    // prevent too large distortions (e.g. when object is at 90deg on the
    // side, then the projection on the image frame is very unstable).
    Eigen::Matrix3d robot_in_world_frame;
    robot_in_world_frame =
        Eigen::AngleAxisd(0.5 * angle_rand(rng), Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(0.5 * angle_rand(rng), Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(2 * M_PI * angle_rand(rng), Eigen::Vector3d::UnitZ());

    // Pose construction.
    Eigen::Quaterniond q(robot_in_world_frame);
    x_view::SE3 pose(q, robot_position);

    const double depth = (object_in_world_coordinates - robot_position).norm();

    x_view::DepthProjector projector(pose, intrinsics);

    const cv::Point2i pixel_coord =
        projector.getPixelCoordinates(object_in_world_coordinates);

    const Eigen::Vector3d new_world_coord =
        projector.getWorldCoordinates(pixel_coord, depth);

    for (int k = 0; k < 3; ++k)
      CHECK_NEAR(object_in_world_coordinates[k], new_world_coord[k], tol);

  }
}

void testPixelToCamera() {
  // Random number generator
  const unsigned long seed = 0;
  const int image_width = 1240;
  const int image_height = 760;
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> rand_x(0, image_width-1);
  std::uniform_int_distribution<int> rand_y(0, image_height-1);

  const int focal_length = 1000;
  Eigen::Vector2i principal_point;
  principal_point << image_width / 2, image_height / 2;
  const double depth = 1.0;
  x_view::CameraIntrinsics intrinsics(focal_length, principal_point);

  const int num_reps = 1000;
  for (int i = 0; i < num_reps; ++i) {
    Eigen::Vector2i pixel;
    pixel << rand_x(rng), rand_y(rng);

    const Eigen::Vector2i diff = pixel - principal_point;
    const double l_x = std::sqrt(diff.x() * diff.x() +
        focal_length * focal_length);
    const double l_y = std::sqrt(diff.y() * diff.y() +
        focal_length * focal_length);

    const double expected_x = diff.x() / l_x * depth;
    const double expected_y = diff.y() / l_y * depth;

    x_view::DepthProjector newProjector(x_view::SE3(), intrinsics);
    Eigen::Vector3d proj = newProjector.pixelToCamera(pixel, depth);

    CHECK_NEAR(proj[0], expected_x, 0.05);
    CHECK_NEAR(proj[1], expected_y, 0.05);
  }
}

}


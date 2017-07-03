#include <x_view_core/landmarks/graph_landmark/projector.h>

namespace x_view {

Projector::Projector(const x_view::SE3& pose,
                    const CameraIntrinsics& intrinsics)
    : pose_(pose),
      intrinsics_(intrinsics) {
  computeProjectionMatrix();
}

Eigen::Vector2i Projector::projectWorldToPixel(
    const Eigen::Vector3d& world_coordinate) const {
  Eigen::Vector4d homo_world;
  homo_world << (world_coordinate - pose_.getPosition()), 1.0;

  Eigen::Vector3d pixel_homo = projection_matrix_ * homo_world;
  return Eigen::Vector2i(pixel_homo[0] / pixel_homo[2],
                         pixel_homo[1] / pixel_homo[2]);
}

Eigen::Vector3d Projector::projectPixelToWorld(
    const Eigen::Vector2i& pixel_coordinates,
    const double depth) const {

  Eigen::Vector4d pixel_homo;
  pixel_homo << pixel_coordinates[0], pixel_coordinates[1], 1.0, 1.0;

  Eigen::Vector4d world_homo = inverse_projection_matrix_ * pixel_homo;

  return  pose_.getPosition() + world_homo.block(0,0,3,1).normalized() * depth;
}


void Projector::computeProjectionMatrix() {
  Eigen::Matrix3d intrinsic_matrix;
  intrinsic_matrix << intrinsics_.focal_length, 0.0, intrinsics_.px,
      0.0, intrinsics_.focal_length, intrinsics_.py,
      0.0, 0.0, 1.0;

  Eigen::MatrixXd extrinsic_matrix =
      pose_.getTransformationMatrix().block(0, 0, 3, 4);

  projection_matrix_ = intrinsic_matrix * extrinsic_matrix;

  Eigen::Matrix4d extended_projection;
  extended_projection << projection_matrix_, 0.0, 0.0, 0.0, 1.0;

  inverse_projection_matrix_ = extended_projection.inverse();
}



}


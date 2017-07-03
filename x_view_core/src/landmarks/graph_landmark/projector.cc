#include <x_view_core/landmarks/graph_landmark/projector.h>

namespace x_view {

Projector::Projector(const x_view::SE3& pose,
                    const CameraIntrinsics& intrinsics)
    : pose_(pose),
      intrinsics_(intrinsics) {
  computeProjectionMatrix();
}

Eigen::Vector3d Projector::projectWorldToCamera(
    const Eigen::Vector3d& world_coordinate) const {
  Eigen::Vector4d world_homo;
  world_homo << world_coordinate, 1.0;

  return extrinsic_matrix_ * world_homo;
}

Eigen::Vector2i Projector::projectCameraToPixel(
    const Eigen::Vector3d& camera_coordinate) const {

  Eigen::Vector3d pixel_homo = intrinsic_matrix_ * camera_coordinate;

  return Eigen::Vector2i(pixel_homo[0]/pixel_homo[2],
  pixel_homo[1] / pixel_homo[2]);

}

Eigen::Vector3d Projector::projectPixelToCamera(
    const Eigen::Vector2i& pixel_coordinate) const {
  Eigen::Vector3d pixel_homo;
  pixel_homo << pixel_coordinate[0], pixel_coordinate[1], 1.0;

  Eigen::Vector3d camera_direction =
      (intrinsic_matrix_.inverse() * pixel_homo).normalized();

  return camera_direction;
}

Eigen::Vector3d Projector::projectCameraToWorld(
    const Eigen::Vector3d camera_coordinate) const {

  const Eigen::Vector3d world_coordinate =
      (pose_.getRotationMatrix() * camera_coordinate).normalized();

  return world_coordinate;
}


void Projector::computeProjectionMatrix() {
  extrinsic_matrix_ = pose_.getTransformationMatrix().inverse().block(0,0,3,4);
  intrinsic_matrix_.setIdentity();
  intrinsic_matrix_(0, 0) = intrinsic_matrix_(1, 1) = intrinsics_.focal_length;
  intrinsic_matrix_(0, 2) = intrinsics_.px;
  intrinsic_matrix_(1, 2) = intrinsics_.py;

}



}


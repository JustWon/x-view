#include <x_view_core/landmarks/graph_landmark/projector.h>

namespace x_view {

Projector::Projector(const x_view::SE3& pose,
                     const CameraIntrinsics& intrinsics,
                     const Eigen::Matrix3d& camera_to_image_rotation)
    : pose_(pose),
      intrinsics_(intrinsics),
      camera_to_image_rotation_(camera_to_image_rotation),
      image_to_camera_rotation_(camera_to_image_rotation_.inverse())
{
  computeIntrinsicMatrix();
}

Eigen::Vector3d Projector::getWorldCoordinates(const cv::Point2i& pixel,
    const double depth) const {
  const Eigen::Vector3d new_camera_coord =
      pixelToCamera(Eigen::Vector2i(pixel.x, pixel.y), depth);

  const Eigen::Vector3d new_world_coord =
      cameraToWorld(new_camera_coord);

  return new_world_coord;
}

cv::Point2i Projector::getPixelCoordinates(const Eigen::Vector3d coordinate) const {
  const Eigen::Vector3d camera_coords =  worldToCamera(coordinate);

  const Eigen::Vector2i pixel_coords = cameraToPixel(camera_coords);

  return cv::Point2i(pixel_coords[0], pixel_coords[1]);
}

Eigen::Vector3d Projector::worldToCamera(
    const Eigen::Vector3d& world_coordinate) const {
    return pose_.inverseTransform(world_coordinate);
}

Eigen::Vector2i Projector::cameraToPixel(
    const Eigen::Vector3d& camera_coordinate) const {

  Eigen::Vector3d image_coordinate =
      camera_to_image_rotation_ * camera_coordinate;
  Eigen::Vector3d pixel_homo = intrinsic_matrix_ * image_coordinate;

  return Eigen::Vector2i(pixel_homo[0]/pixel_homo[2],
                         pixel_homo[1] / pixel_homo[2]);
}

Eigen::Vector3d Projector::pixelToCamera(
    const Eigen::Vector2i& pixel_coordinate, const double depth) const {
  Eigen::Vector3d pixel_homo;
  pixel_homo << pixel_coordinate[0], pixel_coordinate[1], 1.0;

  Eigen::Vector3d camera_direction =
      (inverse_intrinsic_matrix_ * pixel_homo).normalized();

  return image_to_camera_rotation_ * camera_direction * depth;
}

Eigen::Vector3d Projector::cameraToWorld(
    const Eigen::Vector3d& camera_coordinate) const {
  return pose_.transform(camera_coordinate);
}


void Projector::computeIntrinsicMatrix() {
  intrinsic_matrix_.setIdentity();
  intrinsic_matrix_(0, 0) = intrinsic_matrix_(1, 1) = intrinsics_.focal_length;
  intrinsic_matrix_(0, 2) = intrinsics_.px;
  intrinsic_matrix_(1, 2) = intrinsics_.py;

  inverse_intrinsic_matrix_ = intrinsic_matrix_.inverse();
}



}


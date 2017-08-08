#include <x_view_core/landmarks/graph_landmark/depth_projector.h>

namespace x_view {

DepthProjector::DepthProjector(const x_view::SE3& pose,
                     const CameraIntrinsics& intrinsics,
                     const Mat3& camera_to_image_rotation)
    : pose_(pose),
      intrinsic_matrix_(intrinsics.getCameraMatrix()),
      inverse_intrinsic_matrix_(intrinsics.getCameraMatrix().inverse()),
      camera_to_image_rotation_(camera_to_image_rotation),
      image_to_camera_rotation_(camera_to_image_rotation_.inverse()) {
}

Vec3 DepthProjector::getWorldCoordinates(const cv::Point2i& pixel,
                                         const real_t depth) const {
  const Vec3 new_camera_coord =
      pixelToCamera(Eigen::Vector2i(pixel.x, pixel.y), depth);

  const Vec3 new_world_coord = cameraToWorld(new_camera_coord);

  return new_world_coord;
}

cv::Point2i DepthProjector::getPixelCoordinates(const Vec3& coordinate) const {
  const Vec3 camera_coords =  worldToCamera(coordinate);

  const Eigen::Vector2i pixel_coords = cameraToPixel(camera_coords);

  return cv::Point2i(pixel_coords[0], pixel_coords[1]);
}

Vec3 DepthProjector::worldToCamera(const Vec3& world_coordinate) const {
    return pose_.inverseTransform(world_coordinate);
}

Eigen::Vector2i DepthProjector::cameraToPixel(
    const Vec3& camera_coordinate) const {

  Vec3 image_coordinate = camera_to_image_rotation_ * camera_coordinate;
  Vec3 pixel_homo = intrinsic_matrix_ * image_coordinate;

  return Eigen::Vector2i(pixel_homo[0]/pixel_homo[2],
                         pixel_homo[1] / pixel_homo[2]);
}

Vec3 DepthProjector::pixelToCamera( const Eigen::Vector2i& pixel_coordinate,
                                    const real_t depth) const {
  Vec3 pixel_homo;
  pixel_homo << pixel_coordinate[0], pixel_coordinate[1], 1.0;

  Vec3 camera_direction = (inverse_intrinsic_matrix_ * pixel_homo).normalized();

  return image_to_camera_rotation_ * camera_direction * depth;
}

Vec3 DepthProjector::cameraToWorld(const Vec3& camera_coordinate) const {
  return pose_.transform(camera_coordinate);
}

}


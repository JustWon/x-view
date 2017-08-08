#ifndef X_VIEW_DEPTH_PROJECTOR_H
#define X_VIEW_DEPTH_PROJECTOR_H

#include <x_view_core/x_view_types.h>
#include <x_view_core/x_view_locator.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Class responsible for computing the 3D location of a pixel in world
 * space given its depth value and its pixel coordinates.
 */
class DepthProjector {
 public:
  /**
   * \brief Constructor of projector object.
   * \param pose Current robot pose needed to transform between camera and
   * world frame.
   * \param intrinsics Intrinsic camera parameters needed to transform
   * between camera and pixel frame.
   * \param camera_to_image_rotation Rotation matrix describing the rotation
   * involved between the camera frame and the image frame.
   */
  DepthProjector(const SE3& pose, const CameraIntrinsics& intrinsics,
                 const Mat3& camera_to_image_rotation =
                 Locator::getDataset()->getCameraToImageRotation());

  /**
   * \brief Computes the world coordinates of the pixel passed as parameter.
   * \param pixel Pixel of interest.
   * \param depth Depth value associated to the pixel of interest.
   * \return The 3D location of the object projected at the pixel location
   * expressed in the world frame.
   */
  Vec3 getWorldCoordinates(const cv::Point2i& pixel, const real_t depth) const;

  /**
   * \brief Computes the pixel coordinates of a point given in world
   * coordinates.
   * \param coordinate Point expressed in world coordinates to be projected
   * onto the pixel coordinate system.
   * \return Pixel coordinate associated with the 3D position passed as
   * argument.
   */
  cv::Point2i getPixelCoordinates(const Vec3& coordinate) const;

  /// \brief Transforms the 3D point given in world coordinate frame into the
  /// camera frame.
  Vec3 worldToCamera(const Vec3& world_coordinate) const;

  /// \brief Transforms the 3D point given in camera coordinate frame into
  /// pixel coordinates.
  Eigen::Vector2i cameraToPixel(const Vec3& camera_coordinate) const;

  /// \brief Transforms the pixel passed as argument into a 3D point
  /// expressed in camera frame.
  Vec3 pixelToCamera(const Eigen::Vector2i& pixel_coordinate,
                     const real_t depth) const;

  /// \brief Transforms the 3D point give in camera frame into the world frame.
  Vec3 cameraToWorld(const Vec3& camera_coordinate) const;

 private:
  /// \brief Robot's pose expressed in world frame.
  const SE3 pose_;

  /// \brief Intrinsic camera parameters.
  const Mat3 intrinsic_matrix_;
  const Mat3 inverse_intrinsic_matrix_;

  /// \brief Rotation matrix between camera frame (the one described by the
  /// pose_ object) and the image plane.
  const Mat3 camera_to_image_rotation_;
  const Mat3 image_to_camera_rotation_;
};

}

#endif //X_VIEW_DEPTH_PROJECTOR_H

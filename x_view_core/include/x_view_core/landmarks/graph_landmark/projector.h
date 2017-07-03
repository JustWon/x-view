#ifndef X_VIEW_PROJECTOR_H
#define X_VIEW_PROJECTOR_H

#include <x_view_core/x_view_types.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Class responsible for computing the 3D location of a pixel in world
 * space given its depth value and its pixel coordinates.
 */
class Projector {
 public:
  /**
   * \brief Constructor of projector object.
   * \param pose Current robot pose needed to transform between camera and
   * world frame.
   * \param intrinsics Intrinsic camera parameters needed to transform
   * between camera and pixel frame.
   */
  Projector(const SE3& pose, const CameraIntrinsics& intrinsics);

  /**
   * \brief Computes the world coordinates of the pixel passed as parameter.
   * \param pixel Pixel of interest.
   * \param depth Depth value associated to the pixel of interest.
   * \return The 3D location of the object projected at the pixel location
   * expressed in the world frame.
   */
  Eigen::Vector3d getWorldCoordinates(const cv::Point2i& pixel,
                                      const double depth) const;

  /**
   * \brief Computes the pixel coordinates of a point given in world
   * coordinates.
   * \param coordinate Point expressed in world coordinates to be projected
   * onto the pixel coordinate system.
   * \return Pixel coordinate associated with the 3D position passed as
   * argument.
   */
  cv::Point2i getPixelCoordinates(const Eigen::Vector3d coordinate) const;

 private:
  /// \brief Robot's pose expressed in world frame.
  const SE3 pose_;
  /// \brief Intrinsic camera parameters.
  const CameraIntrinsics intrinsics_;

  void computeProjectionMatrix();

  Eigen::Matrix<double, 3, 4> extrinsic_matrix_;
  Eigen::Matrix<double, 3, 3> intrinsic_matrix_;

  /// \brief Transforms the 3D point given in world coordinate frame into the
  /// camera frame.
  Eigen::Vector3d worldToCamera(const Eigen::Vector3d& world_coordinate)
  const;

  /// \brief Transforms the 3D point given in camera coordinate frame into
  /// pixel coordinates.
  Eigen::Vector2i cameraToPixel(const Eigen::Vector3d& camera_coordinate)
  const;

  /// \brief Transforms the pixel passed as argument into a 3D point
  /// expressed in camera frame.
  Eigen::Vector3d pixelToCamera(const Eigen::Vector2i& pixel_coordinate,
                                const double depth) const;

  /// \brief Transforms the 3D point give in camera frame into the world frame.
  Eigen::Vector3d cameraToWorld(const Eigen::Vector3d& camera_coordinate) const;
};

}

#endif //X_VIEW_PROJECTOR_H

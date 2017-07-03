#ifndef X_VIEW_PROJECTOR_H
#define X_VIEW_PROJECTOR_H

#include <x_view_core/x_view_types.h>

#include <Eigen/Core>

namespace x_view {

class Projector {
 public:
  Projector(const SE3& pose, const CameraIntrinsics& intrinsics);

  Eigen::Vector3d projectWorldToCamera(const Eigen::Vector3d& world_coordinate)
  const;

  Eigen::Vector2i projectCameraToPixel(const Eigen::Vector3d& camera_coordinate)
  const;

  Eigen::Vector3d projectPixelToCamera(const Eigen::Vector2i& pixel_coordinate) const;

  Eigen::Vector3d projectCameraToWorld(const Eigen::Vector3d camera_coordinate) const;

 private:
  const SE3 pose_;
  const CameraIntrinsics intrinsics_;

  void computeProjectionMatrix();

  Eigen::Matrix<double, 3, 4> extrinsic_matrix_;
  Eigen::Matrix<double, 3, 3> intrinsic_matrix_;
};

}

#endif //X_VIEW_PROJECTOR_H

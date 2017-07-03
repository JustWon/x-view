#ifndef X_VIEW_PROJECTOR_H
#define X_VIEW_PROJECTOR_H

#include <x_view_core/x_view_types.h>

#include <Eigen/Core>


namespace x_view {

class Projector {
 public:
  Projector(const SE3& pose, const CameraIntrinsics& intrinsics);

  Eigen::Vector2i projectWorldToPixel(const Eigen::Vector3d& world_coordinate)
  const;

  Eigen::Vector3d projectPixelToWorld(const Eigen::Vector2i& pixel_coordinates,
                                      const double depth) const;


 private:
  const SE3 pose_;
  const CameraIntrinsics intrinsics_;

  void computeProjectionMatrix();

  Eigen::Matrix<double, 3, 4> projection_matrix_;
  Eigen::Matrix<double, 4, 4> inverse_projection_matrix_;
};

}

#endif //X_VIEW_PROJECTOR_H

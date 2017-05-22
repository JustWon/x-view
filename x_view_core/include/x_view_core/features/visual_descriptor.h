#ifndef X_VIEW_VISUAL_DESCRIPTOR_H
#define X_VIEW_VISUAL_DESCRIPTOR_H

#include <x_view_core/features/vector_descriptor.h>

#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace x_view {

class VisualDescriptor : public VectorDescriptor {

 public:

  typedef std::vector<cv::KeyPoint> KeyPointsVector;

  VisualDescriptor(const cv::Mat& descriptor, const KeyPointsVector& keypoints);
  virtual ~VisualDescriptor();

  /// \brief Returns a const reference to the keypoints.
  const KeyPointsVector& getKeyPoints() const;

 protected:
  const KeyPointsVector key_points_vector_;
};
}

#endif //X_VIEW_VISUAL_DESCRIPTOR_H

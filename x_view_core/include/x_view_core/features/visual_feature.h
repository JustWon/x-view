#ifndef X_VIRW_VISUAL_FEATURE_H
#define X_VIRW_VISUAL_FEATURE_H

#include <x_view_core/features/vector_feature.h>

#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace x_view {


class VisualFeature : public CVMatFeature {

 public:

  typedef std::vector<cv::KeyPoint> KeyPointsVector;

  VisualFeature(const FeatureRepr& feature, const KeyPointsVector& keypoints);
  virtual ~VisualFeature();

  ///\brief returns a const reference to the keypoints
  const KeyPointsVector& getKeyPoints() const;

 protected:
  const KeyPointsVector key_points_vector_;
};
}

#endif //X_VIRW_VISUAL_FEATURE_H

#include <x_view_core/features/visual_feature.h>

namespace x_view {

VisualFeature::VisualFeature(const cv::Mat& feature,
                             const KeyPointsVector& keypoints)
    : VectorFeature(feature),
      key_points_vector_(keypoints) {
}

VisualFeature::~VisualFeature() {
}

const VisualFeature::KeyPointsVector& VisualFeature::getKeyPoints() const {
  return key_points_vector_;
}

}

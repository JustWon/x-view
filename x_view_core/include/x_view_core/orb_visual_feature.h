#ifndef X_VIEW_ORB_VISUAL_FEATURE_H
#define X_VIEW_ORB_VISUAL_FEATURE_H

#include <x_view_core/visual_feature.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace x_view {

struct ORBVisualFeature : public VisualFeature {

  explicit ORBVisualFeature(const cv::Mat& image, const SE3& pose,
                             const int num_desired_visual_features);

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other) {}

};

}

#endif //X_VIEW_ORB_VISUAL_FEATURE_H

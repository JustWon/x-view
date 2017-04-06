#include <x_view_core/visual_feature.h>

namespace x_view {

VisualFeature::VisualFeature(const cv::Mat& image, const SE3& pose,
                             int num_desired_visual_features)
    : AbstractSemanticLandmark(image, pose),
      num_desired_visual_features_(num_desired_visual_features) {
}
}

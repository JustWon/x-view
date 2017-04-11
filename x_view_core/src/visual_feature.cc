#include <x_view_core/visual_feature.h>

namespace x_view {

// TODO: read number of desired features from config file
int VisualFeature::NUM_VISUAL_FEATURES = 1000;

VisualFeature::VisualFeature(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
}
}

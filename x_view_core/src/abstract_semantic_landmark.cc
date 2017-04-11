#include <x_view_core/abstract_semantic_landmark.h>

namespace x_view {

AbstractSemanticLandmark::AbstractSemanticLandmark(const cv::Mat& image,
                                                   const SE3& pose)
    : image_(image), pose_(pose) {}

AbstractSemanticLandmark::~AbstractSemanticLandmark() {}
}

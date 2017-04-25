#include <x_view_core/landmarks/abstract_semantic_landmark.h>

namespace x_view {

AbstractSemanticLandmark::AbstractSemanticLandmark(const cv::Mat& image,
                                                   const SE3& pose)
    : semantic_image_(image), pose_(pose) {}

AbstractSemanticLandmark::~AbstractSemanticLandmark() {}

const cv::Mat& AbstractSemanticLandmark::getSemanticImage() const {
  return semantic_image_;
}

const SE3& AbstractSemanticLandmark::getPose() const {
  return pose_;
}

const ConstDescriptorPtr& AbstractSemanticLandmark::getDescriptor() const {
  return descriptor_;
}

}
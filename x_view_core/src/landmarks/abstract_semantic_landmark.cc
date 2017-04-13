#include <x_view_core/landmarks/abstract_semantic_landmark.h>

namespace x_view {

AbstractSemanticLandmark::AbstractSemanticLandmark(const cv::Mat& image,
                                                   const SE3& pose)
    : image_(image), pose_(pose) {}

AbstractSemanticLandmark::~AbstractSemanticLandmark() {}

const cv::Mat& AbstractSemanticLandmark::getImage() const {
  return image_;
}

const SE3& AbstractSemanticLandmark::getPose() const {
  return pose_;
}

const ConstFeaturePtr& AbstractSemanticLandmark::getFeature() const {
  return feature_;
}

}
#ifndef X_VIEW_SIFT_VISUAL_FEATURE_H
#define X_VIEW_SIFT_VISUAL_FEATURE_H

#include <x_view_core/visual_feature.h>

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace x_view {

class SIFTVisualFeature : public VisualFeature {

 public:
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {
    return SemanticLandmarkPtr(new SIFTVisualFeature(image, pose));
  }

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other) {}

 protected:
  explicit SIFTVisualFeature(const cv::Mat& image, const SE3& pose);

};

}

#endif //X_VIEW_SIFT_VISUAL_FEATURE_H

#include <x_view_core/visual_feature.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace x_view {

SIFTVisualFeature::SIFTVisualFeature(const cv::Mat& image, const SE3& pose)
    : VisualFeature(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  features_extractor_.reset(new cv::SIFT(NUM_VISUAL_FEATURES));
  features_extractor_->detect(gray, keypoints_);

  // retains only the desired number of features
  if (keypoints_.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints_, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  features_extractor_->compute(gray, keypoints_, descriptors_);
}

SemanticMatchingResult
SIFTVisualFeature::match(const AbstractSemanticLandmark& other) {

}

}

#include <x_view_core/landmarks/visual_feature.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace x_view {

SURFVisualFeature::SURFVisualFeature(const cv::Mat& image, const SE3& pose)
    : VisualFeature(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  const double hessian_threshold = 0.2;
  features_extractor_.reset(new cv::SURF(hessian_threshold));
  features_extractor_->detect(gray, keypoints_);

  // retains only the desired number of features
  if (keypoints_.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints_, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  features_extractor_->compute(gray, keypoints_, descriptors_);
}


}

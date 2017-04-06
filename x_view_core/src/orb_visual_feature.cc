#include <x_view_core/orb_visual_feature.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace x_view {

ORBVisualFeature::ORBVisualFeature(const cv::Mat& image, const SE3& pose,
                                     const int num_desired_visual_features)
    : VisualFeature(image, pose, num_desired_visual_features) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  features_extractor_.reset(new cv::ORB(num_desired_visual_features_));
  features_extractor_->detect(gray, keypoints_);

  // retains only the desired number of features
  if(keypoints_.size() > num_desired_visual_features)
    cv::KeyPointsFilter::retainBest(keypoints_, num_desired_visual_features_);

  // compute the feature descriptors
  features_extractor_->compute(gray, keypoints_, descriptors_);
}

}

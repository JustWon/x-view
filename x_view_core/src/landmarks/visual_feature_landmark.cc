#include <x_view_core/landmarks/visual_feature_landmark.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace x_view {

// TODO: read number of desired features from config file
int VisualFeatureLandmark::NUM_VISUAL_FEATURES = 1000;

VisualFeatureLandmark::VisualFeatureLandmark(const cv::Mat& image,
                                             const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
}


//**************************** ORB visual feature ****************************//

ORBVisualFeatureLandmark::ORBVisualFeatureLandmark(const cv::Mat& image,
                                                   const SE3& pose)
    : VisualFeatureLandmark(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  features_extractor_.reset(new cv::ORB(NUM_VISUAL_FEATURES));
  features_extractor_->detect(gray, keypoints_);

  // retains only the desired number of features
  if (keypoints_.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints_, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  features_extractor_->compute(gray, keypoints_, descriptors_);
}

//**************************** SIFT visual feature ***************************//

SIFTVisualFeatureLandmark::SIFTVisualFeatureLandmark(const cv::Mat& image,
                                                     const SE3& pose)
    : VisualFeatureLandmark(image, pose) {

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

//**************************** SURF visual feature ***************************//

SURFVisualFeatureLandmark::SURFVisualFeatureLandmark(const cv::Mat& image,
                                                     const SE3& pose)
    : VisualFeatureLandmark(image, pose) {

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

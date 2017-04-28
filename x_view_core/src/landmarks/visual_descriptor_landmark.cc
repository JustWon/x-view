#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <x_view_core/landmarks/visual_descriptor_landmark.h>
#include <x_view_core/features/visual_descriptor.h>


namespace x_view {

// TODO: read number of desired features from config file
int VisualDescriptorLandmark::NUM_VISUAL_FEATURES = 1000;

VisualDescriptorLandmark::VisualDescriptorLandmark(const cv::Mat& image,
                                             const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
}


//*************************** ORB visual descriptor **************************//

ORBVisualDescriptorLandmark::ORBVisualDescriptorLandmark(const cv::Mat& image,
                                                   const SE3& pose)
    : VisualDescriptorLandmark(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  features_extractor_.reset(new cv::ORB(NUM_VISUAL_FEATURES));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor_->detect(gray, keypoints);

  // retains only the desired number of features
  if (keypoints.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor_->compute(gray, keypoints, descriptors);

  descriptor_ =
      std::make_shared<VisualDescriptor>(VisualDescriptor(descriptors, keypoints));
}

//*************************** SIFT visual descriptor *************************//

SIFTVisualDescriptorLandmark::SIFTVisualDescriptorLandmark(const cv::Mat& image,
                                                     const SE3& pose)
    : VisualDescriptorLandmark(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  features_extractor_.reset(new cv::SIFT(NUM_VISUAL_FEATURES));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor_->detect(gray, keypoints);

  // retains only the desired number of features
  if (keypoints.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor_->compute(gray, keypoints, descriptors);

  descriptor_ =
      std::make_shared<VisualDescriptor>(VisualDescriptor(descriptors, keypoints));
}

//************************** SURF visual descriptor **************************//

SURFVisualDescriptorLandmark::SURFVisualDescriptorLandmark(const cv::Mat& image,
                                                     const SE3& pose)
    : VisualDescriptorLandmark(image, pose) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  const double hessian_threshold = 0.2;
  features_extractor_.reset(new cv::SURF(hessian_threshold));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor_->detect(gray, keypoints);

  // retains only the desired number of features
  if (keypoints.size() > NUM_VISUAL_FEATURES)
    cv::KeyPointsFilter::retainBest(keypoints, NUM_VISUAL_FEATURES);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor_->compute(gray, keypoints, descriptors);

  descriptor_ =
      std::make_shared<VisualDescriptor>(VisualDescriptor(descriptors, keypoints));
}

}

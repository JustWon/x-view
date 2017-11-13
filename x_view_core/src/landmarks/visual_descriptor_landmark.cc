#include <x_view_core/landmarks/visual_descriptor_landmark.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/visual_descriptor.h>
#include <x_view_core/x_view_locator.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace x_view {

int VisualDescriptorLandmark::DEFAULT_NUM_VISUAL_FEATURES = 1000;
real_t VisualDescriptorLandmark::DEFAULT_HESSIAN_THRESHOLD = 0.2;

VisualDescriptorLandmark::VisualDescriptorLandmark(
    const FrameData& frame_data)
    : AbstractSemanticLandmark(frame_data) {
}


//*************************** ORB visual descriptor **************************//

ORBVisualDescriptorLandmark::ORBVisualDescriptorLandmark(
    const FrameData& frame_data)
    : VisualDescriptorLandmark(frame_data) {

  const auto& dataset = Locator::getDataset();
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");

  const int num_visual_features =
      landmark_parameters->getInteger("num_visual_features",
                                      DEFAULT_NUM_VISUAL_FEATURES);

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(semantic_image_, gray, CV_BGR2GRAY);

  features_extractor.reset(new cv::ORB(num_visual_features));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor->detect(
      gray * 255. / dataset->numSemanticClasses(), keypoints);

  // retains only the desired number of features
  if (keypoints.size() > num_visual_features)
    cv::KeyPointsFilter::retainBest(keypoints, num_visual_features);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor->compute(gray, keypoints, descriptors);

  descriptor_ = std::make_shared<VisualDescriptor>(
      VisualDescriptor(descriptors, keypoints));
}

//*************************** SIFT visual descriptor *************************//

SIFTVisualDescriptorLandmark::SIFTVisualDescriptorLandmark(
    const FrameData& frame_data)
    : VisualDescriptorLandmark(frame_data) {

  const auto& dataset = Locator::getDataset();
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");

  const int num_visual_features =
      landmark_parameters->getInteger("num_visual_features",
                                      DEFAULT_NUM_VISUAL_FEATURES);

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(semantic_image_, gray, CV_BGR2GRAY);

  features_extractor.reset(new cv::SIFT(num_visual_features));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor->detect(
      gray * 255. / dataset->numSemanticClasses(), keypoints);

  // retains only the desired number of features
  if (keypoints.size() > num_visual_features)
    cv::KeyPointsFilter::retainBest(keypoints, num_visual_features);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor->compute(gray, keypoints, descriptors);

  descriptor_ = std::make_shared<VisualDescriptor>(
      VisualDescriptor(descriptors, keypoints));
}

//************************** SURF visual descriptor **************************//

SURFVisualDescriptorLandmark::SURFVisualDescriptorLandmark(
    const FrameData& frame_data)
    : VisualDescriptorLandmark(frame_data) {

  const auto& dataset = Locator::getDataset();
  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");

  const int num_visual_features =
      landmark_parameters->getInteger("num_visual_features",
                                      DEFAULT_NUM_VISUAL_FEATURES);
  const real_t hessian_threshold =
      landmark_parameters->getFloat("hessian_threshold",
                                    DEFAULT_HESSIAN_THRESHOLD);

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(semantic_image_, gray, CV_BGR2GRAY);

  features_extractor.reset(new cv::SURF(hessian_threshold));

  std::vector<cv::KeyPoint> keypoints;
  features_extractor->detect(
      gray * 255. / dataset->numSemanticClasses(), keypoints);

  // retains only the desired number of features
  if (keypoints.size() > num_visual_features)
    cv::KeyPointsFilter::retainBest(keypoints, num_visual_features);

  // compute the feature descriptors
  cv::Mat descriptors;
  features_extractor->compute(gray, keypoints, descriptors);

  descriptor_ = std::make_shared<VisualDescriptor>(
      VisualDescriptor(descriptors, keypoints));
}

}

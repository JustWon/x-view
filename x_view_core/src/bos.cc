#include <x_view_core/bos.h>

#include <glog/logging.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace x_view {

BoS::BoS(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
}

SemanticMatchingResult BoS::match(const AbstractSemanticLandmark& other) {
  CHECK(false) << "Not implemented yet";
}

BoSVisualFeatures::BoSVisualFeatures(const cv::Mat& image, const SE3& pose,
                                     const int num_desired_visual_features)
    : BoS(image, pose),
      num_desired_visual_features_(num_desired_visual_features) {

  // convert RGB image to gray as visual feature detectors only work on
  // single-channel images
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);


  features_extractor_ = cv::Feature2D::create("ORB");
  features_extractor_->detect(gray, keypoints_);

  // retains only the desired number of features
  cv::KeyPointsFilter::retainBest(keypoints_, num_desired_visual_features_);

  features_extractor_->compute(gray, keypoints_, descriptors_);

  std::cout << "\t:==\tComputed " << descriptors_.rows << " descriptors of "
      "length "
      "" << descriptors_.cols << " from " << keypoints_.size() << " "
                "keypoints" << std::endl;
}
}

#include <x_view_core/matchers/vector_features_matcher.h>
#include <x_view_core/landmarks/visual_feature_landmark.h>
#include <x_view_core/features/vector_feature.h>
#include <highgui.h>

namespace x_view {

VectorFeaturesMatcher::VectorFeaturesMatcher()
    : num_retained_best_matches_(1) {
  descriptor_matcher_ =
      std::shared_ptr<cv::DescriptorMatcher>(new cv::BFMatcher);
}

VectorFeaturesMatcher::~VectorFeaturesMatcher() {
}

void VectorFeaturesMatcher::addLandmark(const SemanticLandmarkPtr& landmark) {

  std::shared_ptr<const CVMatFeature> vecFeature =
      std::dynamic_pointer_cast<const CVMatFeature>(landmark->getFeature());

  descriptor_matcher_->add(std::vector<cv::Mat>{vecFeature->getFeature()});
}

void VectorFeaturesMatcher::match(const SemanticLandmarkPtr& queryLandmark,
                                  MatchingResultPtr& matchingResult) {

  auto vectorFeature =
      std::dynamic_pointer_cast<const CVMatFeature>(queryLandmark->getFeature());

  const cv::Mat& feature = vectorFeature->getFeature();

  auto result =
      std::dynamic_pointer_cast<VectorMatchingResult>(matchingResult);
  result->matches.resize(100);
  std::cout << "Matches size: " << result->matches.size()<<std::endl;

  descriptor_matcher_->knnMatch(feature, result->matches,
                                num_retained_best_matches_);

}

LandmarksMatcherPtr VectorFeaturesMatcher::create() {
  return LandmarksMatcherPtr(new VectorFeaturesMatcher());
}

}


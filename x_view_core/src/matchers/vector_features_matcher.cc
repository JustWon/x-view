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

  std::shared_ptr<const VectorFeature> vecFeature =
      std::dynamic_pointer_cast<const VectorFeature>(landmark->getFeature());

  descriptor_matcher_->add(std::vector<cv::Mat>{vecFeature->getFeature()});

  std::cout << "Managed to add landmark" << std::endl;
}

void VectorFeaturesMatcher::match(const SemanticLandmarkPtr& queryLandmark,
                                  MatchingResultPtr& matchingResult) {

  auto vectorFeature =
      std::dynamic_pointer_cast<const VectorFeature>(queryLandmark->getFeature());

  std::shared_ptr<VectorMatchingResult>
      vMatchingResult(new VectorMatchingResult);

  descriptor_matcher_->knnMatch(vectorFeature->getFeature(),
                                vMatchingResult->matches,
                                num_retained_best_matches_);

  matchingResult = vMatchingResult;

}

LandmarksMatcherPtr VectorFeaturesMatcher::create() {
  return LandmarksMatcherPtr(new VectorFeaturesMatcher());
}

}


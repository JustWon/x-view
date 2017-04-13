#include <x_view_core/matchers/vector_features_matcher.h>

namespace x_view {

VectorFeaturesMatcher::VectorFeaturesMatcher()
    : num_retained_best_matches_(1) {
  descriptor_matcher_ =
      std::shared_ptr<cv::DescriptorMatcher>(new cv::BFMatcher);
}

VectorFeaturesMatcher::~VectorFeaturesMatcher() {
}

void VectorFeaturesMatcher::add_descriptor(const cv::Mat& descriptor) {
  // forward the call to the cv::BFMatcher
  descriptor_matcher_->add(std::vector<cv::Mat>{descriptor});
}

void VectorFeaturesMatcher::match(const cv::Mat& queryDescriptor,
                                  MatchingResult& matches) {

  descriptor_matcher_->knnMatch(queryDescriptor,
                                matches,
                                num_retained_best_matches_);
}

LandmarksMatcherPtr VectorFeaturesMatcher::create() {
  return LandmarksMatcherPtr(new VectorFeaturesMatcher());
}

}


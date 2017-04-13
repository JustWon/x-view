#include <x_view_core/matchers/vector_features_matcher.h>

namespace x_view {

VectorFeaturesMatcher::VectorFeaturesMatcher()
    : num_retained_best_matches_(1) {
  descriptor_matcher_ =
      std::shared_ptr<cv::DescriptorMatcher>(new cv::BFMatcher);
}

VectorFeaturesMatcher::~VectorFeaturesMatcher() {
}

void VectorFeaturesMatcher::add_descriptor(cv::Mat descriptor) {
  std::vector<cv::Mat> v(1);
  v[0] = descriptor;
  descriptor_matcher_->add(v);
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


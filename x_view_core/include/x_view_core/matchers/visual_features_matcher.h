#ifndef X_VIEW_VISUAL_FEATURES_MATCHER_H
#define X_VIEW_VISUAL_FEATURES_MATCHER_H

#include <x_view_core/x_view_types.h>
#include <x_view_core/matchers/vector_features_matcher.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <memory>

namespace x_view {

class VisualFeaturesMatcher : public VectorFeaturesMatcher {

 public:
  VisualFeaturesMatcher();
  virtual ~VisualFeaturesMatcher();

  virtual void add_descriptor(cv::Mat descriptor);

  virtual void match(const cv::Mat& queryDescriptor,
                     MatchingResult& matches);

  static LandmarksMatcherPtr create();

 protected:
  std::shared_ptr<cv::DescriptorMatcher> descriptor_matcher_;
  const int num_retained_best_matches_;
};

}

#endif //X_VIEW_VISUAL_FEATURES_MATCHER_H

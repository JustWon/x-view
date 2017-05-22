#ifndef X_VIEW_VECTOR_MATCHER_H
#define X_VIEW_VECTOR_MATCHER_H

#include <x_view_core/matchers/abstract_matcher.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <memory>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on feature vectors
 * must implement.
 */
class VectorMatcher : public AbstractMatcher {

 public:

  VectorMatcher();
  virtual ~VectorMatcher();

  class VectorMatchingResult : public AbstractMatchingResult {
   public:
    VectorMatchingResult() {}
    virtual ~VectorMatchingResult() {}

    typedef std::vector<std::vector<cv::DMatch>> Matches;
    Matches matches;

  };

  virtual MatchingResultPtr match(const SemanticLandmarkPtr& query_landmark)
  override;

  static LandmarksMatcherPtr create();

 protected:
  std::shared_ptr<cv::DescriptorMatcher> descriptor_matcher_;
  const int num_retained_best_matches_;

};

}

#endif //X_VIEW_VECTOR_MATCHER_H

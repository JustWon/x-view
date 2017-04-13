#ifndef X_VIEW_VECTOR_FEATURES_MATCHER_H
#define X_VIEW_VECTOR_FEATURES_MATCHER_H

#include <x_view_core/x_view_types.h>
#include <x_view_core/matchers/abstrac_landmarks_matcher.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>
#include <memory>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on feature vectors
 * must implement
 */
class VectorFeaturesMatcher : public AbstractLandmarksMatcher {

 public:

  typedef std::vector<std::vector<cv::DMatch>> MatchingResult;

  VectorFeaturesMatcher();
  virtual ~VectorFeaturesMatcher();

  virtual void addLandmark(const SemanticLandmarkPtr& landmark);

  /**
   * \brief Matches a new descriptor to the ones stored in its internal
   * representation.
   * \param queryDescriptor the new descriptor to be matched
   * \param matchingResult result of the matching
   * \details matchingResult is a vector of N elements, where N is equal to
   * the number of features (rows) of queryDescriptors. Each element 'i' of the
   * vector consists in an other small vector of predefined size (usually 2)
   * which contains the best matches for the 'i'-th feature (row) of the
   * queryDescriptor. Those matches are represented as cv::DMatch objects,
   * which contain a reference to the matched image
   */
  virtual void match(const cv::Mat& queryDescriptor,
                     MatchingResult& matchingResult);

  static LandmarksMatcherPtr create();

 protected:
  std::shared_ptr<cv::DescriptorMatcher> descriptor_matcher_;
  const int num_retained_best_matches_;

};

}

#endif //X_VIEW_VECTOR_FEATURES_MATCHER_H

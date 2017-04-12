#ifndef X_VIEW_VECTOR_FEATURES_MATCHER_H
#define X_VIEW_VECTOR_FEATURES_MATCHER_H

#include <x_view_core/matchers/abstrac_landmarks_matcher.h>

#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on feature vectors
 * must implement
 */
class VectorFeaturesMatcher : public AbstractLandmarksMatcher {

 public:

  typedef std::vector<std::vector<cv::DMatch>> MatchingResult;

  VectorFeaturesMatcher() {}
  virtual ~VectorFeaturesMatcher() {}

  /**
   * \brief Prototypical function each landmarksmatcher based on vector
   * features must implement. This function adds a descriptor (cv::Mat to
   * its internal representation
   * \param descriptor a descriptor to be added to the internal
   * representation of the matcher
   */
  virtual void add_descriptor(cv::Mat descriptor) = 0;

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
                     MatchingResult& matchingResult) = 0;

};

}

#endif //X_VIEW_VECTOR_FEATURES_MATCHER_H

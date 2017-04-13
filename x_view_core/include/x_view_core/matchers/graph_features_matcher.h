#ifndef X_VIEW_GRAPH_FEATURES_MATCHER_H
#define X_VIEW_GRAPH_FEATURES_MATCHER_H

#include <x_view_core/matchers/abstrac_landmarks_matcher.h>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on graph-features
 * must implement
 */
class GraphFeaturesMatcher : public AbstractLandmarksMatcher {

 public:

  //FIXME: clearly the result is not a double
  typedef double MatchingResult;

  // FIXME: clearly a semantic graph is not represented as a double
  typedef double Graph;

  GraphFeaturesMatcher() {}
  virtual ~GraphFeaturesMatcher() {}

  // FIXME: either pass LandmarkPointer or create a base class of descriptors
  // such that all matchers have the same interface defined in the
  // abstractLandmarksMatcher class
  /**
   * \brief Prototypical function each landmarksmatcher based on vector
   * features must implement. This function adds a descriptor (cv::Mat to
   * its internal representation
   * \param descriptor a descriptor to be added to the internal
   * representation of the matcher
   */
  virtual void add_descriptor(Graph descriptor) {};

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
  virtual void match(const Graph& queryDescriptor,
                     MatchingResult& matchingResult) {};

};

}

#endif //X_VIEW_GRAPH_FEATURES_MATCHER_H

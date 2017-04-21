#ifndef X_VIEW_GRAPH_MATCHER_H
#define X_VIEW_GRAPH_MATCHER_H

#include <x_view_core/matchers/abstrac_matcher.h>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on graph-features
 * must implement
 */
class GraphMatcher : public AbstractMatcher {

 public:

  GraphMatcher() {}
  virtual ~GraphMatcher() {}

  class GraphMatchingResult : public AbstractMatchingResult {

  };

  virtual void addLandmark(const SemanticLandmarkPtr& landmark) {}

  virtual void match(const SemanticLandmarkPtr& queryLandmark,
                     MatchingResultPtr& matchingResult) {}
};

}

#endif //X_VIEW_GRAPH_MATCHER_H

#ifndef X_VIEW_GRAPH_MATCHER_H
#define X_VIEW_GRAPH_MATCHER_H

#include <x_view_core/matchers/abstract_matcher.h>

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

  virtual MatchingResultPtr match(const SemanticLandmarkPtr& queryLandmark) override {
    return std::make_shared<GraphMatchingResult>();
  }
};

}

#endif //X_VIEW_GRAPH_MATCHER_H

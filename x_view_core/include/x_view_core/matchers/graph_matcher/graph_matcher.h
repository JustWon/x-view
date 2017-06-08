#ifndef X_VIEW_GRAPH_MATCHER_H
#define X_VIEW_GRAPH_MATCHER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/abstract_matcher.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/x_view_types.h>

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <vector>

namespace x_view {

/**
 * \brief An interface each landmark-matcher based on graph-features
 * must implement.
 */
class GraphMatcher : public AbstractMatcher {

 public:

  GraphMatcher();
  virtual ~GraphMatcher();

  class GraphMatchingResult : public AbstractMatchingResult {
   public:
    GraphMatchingResult()
        : AbstractMatchingResult(),
          similarity_matrix_() {
    }

    GraphMatchingResult(const Eigen::MatrixXf& similarity_matrix)
        : AbstractMatchingResult(),
          similarity_matrix_(similarity_matrix) {
    }

    const Eigen::MatrixXf& getSimilarityMatrix() const {
      return similarity_matrix_;
    }

   private:
    Eigen::MatrixXf similarity_matrix_;
  };

  virtual MatchingResultPtr match(const SemanticLandmarkPtr& query_landmark)
  override;

  static LandmarksMatcherPtr create();

 private:
  Graph global_semantic_graph_;
  std::vector<RandomWalker::WalkMap> global_walk_map_vector_;

};

}

#endif //X_VIEW_GRAPH_MATCHER_H

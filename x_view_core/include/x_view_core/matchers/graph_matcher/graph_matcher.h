#ifndef X_VIEW_GRAPH_MATCHER_H
#define X_VIEW_GRAPH_MATCHER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/abstract_matcher.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>
#include <x_view_core/x_view_types.h>

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <vector>

namespace x_view {

/**
 * \brief A class that matches graph landmarks with the database semantic graph.
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

    const Eigen::MatrixXf& getSimilarityMatrix() const {
      return similarity_matrix_;
    }

    Eigen::MatrixXf& getSimilarityMatrix() {
      return similarity_matrix_;
    }

   private:
    Eigen::MatrixXf similarity_matrix_;
  };

  virtual MatchingResultPtr match(const SemanticLandmarkPtr& query_landmark)
  override;

  static LandmarksMatcherPtr create();

  /**
   * \brief Computes the similarity matrix between all vertices belonging to
   * the global_semantic_graph and the vertices contained in the query graph
   * passed as argument.
   * \param random_walker RandomWalker object initialized with random walks
   * of the query_graph.
   * \param similarity_matrix Computed matrix where each element (i,j)
   * corresponds to the similarity computed between the i-th vertex of the
   * global_semantic_graph_ and the j-th vertex of the query_graph passed as
   * argument.
   * \param score_type Flag indicating which score type must be used when
   * computing the pairwise similarity between vertices.
   */
  void computeSimilarityMatrix(const RandomWalker& random_walker,
                               Eigen::MatrixXf* similarity_matrix,
                               const VertexSimilarity::SCORE_TYPE score_type =
                               VertexSimilarity::SCORE_TYPE::HARD);

 private:
  Graph global_semantic_graph_;
  std::vector<RandomWalker::WalkMap> global_walk_map_vector_;

};

}

#endif //X_VIEW_GRAPH_MATCHER_H

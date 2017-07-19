#ifndef X_VIEW_GRAPH_MATCHER_H
#define X_VIEW_GRAPH_MATCHER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/abstract_matcher.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>
#include <x_view_core/x_view_types.h>

#include <Eigen/Sparse>

#include <map>
#include <memory>
#include <vector>

namespace x_view {

typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    TransformationVector;
typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

/**
 * \brief A class that matches graph landmarks with the database semantic graph.
 */
class GraphMatcher : public AbstractMatcher {

 public:

  typedef Eigen::MatrixXf SimilarityMatrixType;
  typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>
      MaxSimilarityMatrixType;

  GraphMatcher();
  GraphMatcher(const RandomWalkerParams& random_walker_params,
               const VertexSimilarity::SCORE_TYPE score_type);

  virtual ~GraphMatcher();

  class GraphMatchingResult : public AbstractMatchingResult {
   public:
    GraphMatchingResult()
        : AbstractMatchingResult(),
          similarity_matrix_(), invalid_matches_() {
    }

    const SimilarityMatrixType& getSimilarityMatrix() const {
      return similarity_matrix_;
    }

    SimilarityMatrixType& getSimilarityMatrix() {
      return similarity_matrix_;
    }

    const VectorXb& getInvalidMatches() const {
      return invalid_matches_;
    }

    VectorXb& getInvalidMatches() {
      return invalid_matches_;
    }

    MaxSimilarityMatrixType computeMaxSimilarityColwise() const;
    MaxSimilarityMatrixType computeMaxSimilarityRowwise() const;

   private:
    SimilarityMatrixType similarity_matrix_;
    VectorXb invalid_matches_;
  };

  virtual MatchingResultPtr match(const SemanticLandmarkPtr& query_landmark)
  override;

  /**
   * \brief Overloaded function that directly matches the passed Graph to the
   * global_semantic_graph_ stored in this matcher instance.
   * \param query_semantic_graph Semantic graph to be matched against the
   * global_semantic_graph_.
   * \return MatchingResultPtr containing the result of the matching.
   */
  MatchingResultPtr match(const Graph& query_semantic_graph);

  /**
   * \brief Function that filters the Matches between graphs for geometric
   * consistency.
   * \param query_semantic_graph Semantic graph that was matched against the
   * global_semantic_graph_.
   * \param matches the initially established matches.
   * \param filtered matches the geometrically filtered matches.
   * \return bool indication of successful filtering.
   */
  bool filter_matches(const Graph& query_semantic_graph,
                      const Graph& database_semantic_graph,
                      const GraphMatchingResult& matches,
                      VectorXb* invalid_matches);

  virtual void addDescriptor(const ConstDescriptorPtr& descriptor) override;

  /**
   * \brief Overloaded function that directly adds the passed Graph to the
   * matcher.
   * \param graph Graph to be added to the matcher.r
   * associated to the graph passed as first argument.
   */
  void addDescriptor(const Graph& graph);

  static LandmarksMatcherPtr create();

  static LandmarksMatcherPtr create(const RandomWalkerParams& random_walker_params,
                                    const VertexSimilarity::SCORE_TYPE score_type);


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
                               SimilarityMatrixType* similarity_matrix,
                               VectorXb* invalid_matches,
                               const VertexSimilarity::SCORE_TYPE score_type =
                               VertexSimilarity::SCORE_TYPE::WEIGHTED) const;

  const Graph& getGlobalGraph() const  { return global_semantic_graph_; }
  void setGlobalGraph(const Graph& global_graph) {
    global_semantic_graph_ = global_graph;
  }

 private:
  Graph global_semantic_graph_;
  std::vector<RandomWalker::WalkMap> global_walk_map_vector_;

  RandomWalkerParams random_walker_params_;
  VertexSimilarity::SCORE_TYPE vertex_similarity_score_type_;
};

}

#endif //X_VIEW_GRAPH_MATCHER_H

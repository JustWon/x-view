#ifndef X_VIEW_GRAPH_MERGER_H
#define X_VIEW_GRAPH_MERGER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/graph_matcher.h>

#include <queue>

namespace x_view {

/**
 * \brief This class implements the routine for merging two semantic graphs
 * based on the semantic similarity between the graph vertices.
 */
class GraphMerger {

  /// \brief Datastructure used to keep track of which vertices of the query
  /// graph still need to be "linked" to the database graph.
  typedef std::queue<VertexDescriptor> DescriptorQueue;

 public:

  /**
   * \brief Constructor of the graph merger instance.
   * \param database_graph Graph structure representing the database graph.
   * \param query_graph Graph structure representing the query graph to be
   * merged into the database graph.
   * \param matching_result Object resulting from the matching between the
   * query and the database graph. The similarity matrix contained in this
   * structure is used as mean for deciding which vertex of the query graph
   * is associated with which vertex of the database graph.
   */
  GraphMerger(const Graph& database_graph, const Graph& query_graph,
              const GraphMatcher::GraphMatchingResult& matching_result);

  /**
   * \brief Performs the merging operation between the query and the database
   * graph.
   * \return A new graph representing the merged version of the query and the
   * database graph.
   * \details The database graph is usually much larger than the query graph.
   * For this reason, the graph resulting from merging is initialized as a
   * copy of the database graph and extended with the vertices of the query
   * graph.
   */
  const Graph computeMergedGraph();

 private:
  const Graph& database_graph_;
  const Graph& query_graph_;
  const GraphMatcher::GraphMatchingResult& matching_result_;

  /// \brief Graph resulting from the merging operation.
  Graph merged_graph_;

  /// \brief Queue containing all vertex descriptors of the query graph which
  /// still need to be merged into the database graph.
  DescriptorQueue still_to_process_;

  /// \brief This map maps the vertices of the query graph to the associated
  /// vertices in the database graph.
  std::unordered_map<VertexDescriptor, VertexDescriptor> query_in_db_;

  /// \brief Index assigned to the vertices being added to the merged graph.
  int current_vertex_index_;

  /// \brief Vector containign references to vertices belonging to the query
  /// graph which have been matched to a vertex of the database graph.
  std::vector<VertexDescriptor> matched_vertices_;

  GraphMatcher::MaxSimilarityMatrixType computeAgreementMatrix() const;

  void addVertexToMergedGraph(const VertexDescriptor& source_in_query_graph);

};

}

#endif //X_VIEW_GRAPH_MERGER_H

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
   * \param time_window Integer referring to time window allowed to perform
   * merges. In particular, if time_window == 0, no candidate merge is
   * discarded. In the other hand, given time_window > 0, only candidate
   * merges between vertices tagged at most time_windows far apart are merged.
   */
  GraphMerger(const Graph& database_graph, const Graph& query_graph,
              const GraphMatcher::GraphMatchingResult& matching_result,
              const uint64_t time_window = 0);

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
  /// \brief Const reference to the database graph.
  const Graph& database_graph_;

  /// \brief Const reference to the query graph to be merged into the
  /// database graph.
  const Graph& query_graph_;

  /// \brief Structure containing information about the similarities between
  /// the vertices in the query and in the database graph.
  const GraphMatcher::GraphMatchingResult& matching_result_;

  /// \brief Time window used to select which candidate matching vertices
  /// should be merged together. In particular, if a vertex is candidate to
  /// be merged with another, the merge only takes place if the time-distance
  /// between the two vertices is smaller or equal to the allowed time window
  /// defind by time_window_.
  const uint64_t time_window_;

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

  /**
   * \brief Computes the temporal distance between the i-th vertex of the
   * database graph and the j-th vertex of the query graph.
   * \param i Index of the database vertex to be considered.
   * \param j Index of the query vertex to be considered.
   * \return The temporal distance between the two vertices indicated by the
   * indices passed as argument. This value is computed by subtracting the
   * 'last_time_seen' value of the database vertex from the 'last_time_seen'
   * value of the query vertex.
   */
  const uint64_t temporalDistance(const uint64_t i, const uint64_t j) const;

  /**
   * \brief Computes the euclidean distance between the i-th vertex of the
   * database graph and the j-th vertex of the query graph.
   * \param i Index of the database vertex to be considered.
   * \param j Index of the query vertex to be considered.
   * \return The euclidean distance between the two vertices indicated by the
   * indices passed as argument. This value is computed by subtracting the
   * 'location_3d' property of the database vertex from the 'location_3d'
   * value of the query vertex and by taking its norm.
   */
  const double spatialDistance(const uint64_t i, const uint64_t j) const;

};

}

#endif //X_VIEW_GRAPH_MERGER_H

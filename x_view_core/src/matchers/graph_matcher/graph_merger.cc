#include <x_view_core/matchers/graph_matcher/graph_merger.h>

#include <x_view_core/matchers/graph_matcher.h>

namespace x_view {

GraphMerger::GraphMerger(const Graph& database_graph,
                         const Graph& query_graph,
                         const GraphMatcher::GraphMatchingResult& matching_result,
                         const GraphMergerParameters& graph_merger_parameters)
    : database_graph_(database_graph),
      query_graph_(query_graph),
      matching_result_(matching_result),
      graph_merger_parameters_(graph_merger_parameters) {

  LOG(INFO) << "Using graph merger with following parameters:"
      << "\n\tTime window:          "
      << graph_merger_parameters_.time_window
      << "\n\tSimilarity threshold: "
      << graph_merger_parameters_.similarity_threshold
      << "\n\tDistance threshold:   "
      << graph_merger_parameters_.distance_threshold << ".";

}

const Graph GraphMerger::computeMergedGraph() {

  const uint64_t num_query_vertices = boost::num_vertices(query_graph_);
  const uint64_t num_db_vertices = boost::num_vertices(database_graph_);

  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      matching_result_.getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      computeAgreementMatrix();

  // Initialize the merged graph to contain all vertices of the database_graph.
  merged_graph_ = database_graph_;

  // This map contains the matches between the vertex descriptors in the
  // query graph to the corresponding vertex descriptor in the database_graph.
  query_in_db_.clear();

  // Loop over the vertices of the query graph.
  for(int j = 0; j < num_query_vertices; ++j) {
    // Loop over the vertices of the database graph.
    for(int i = 0; i < num_db_vertices; ++i) {
      // Check if the two vertices are possible matches.
      if(max_similarity_agree(i, j) == true
          && similarity_matrix(i, j) >= graph_merger_parameters_.similarity_threshold &&
          temporalDistance(i, j) <= graph_merger_parameters_.time_window &&
          spatialDistance(i, j) <= graph_merger_parameters_.distance_threshold) {
        // There is a match between the j-th vertex of the query graph and
        // the i-th vertex of the database graph.
        query_in_db_.insert({j, i});
        matched_vertices_.push_back(j);
        break;
      }
    }
  }

  // Keep two disconnected graphs??
  if(matched_vertices_.size() == 0) {
    LOG(ERROR) << "Zero matched vertices!";
  }

  // Push all matched vertices into the queue of 'still to process' vertices.
  still_to_process_ = DescriptorQueue();
  for (const VertexDescriptor v_d : matched_vertices_) {
    still_to_process_.push(v_d);
  }

  // Define start vertex index of vertices being added to the merged graph.
  current_vertex_index_ = std::numeric_limits<int>::min();
  const auto db_vertices = boost::vertices(database_graph_);
  for (auto iter = db_vertices.first; iter != db_vertices.second; ++iter)
    current_vertex_index_ =
        std::max(current_vertex_index_, database_graph_[*iter].index);
  ++current_vertex_index_;


  // Iterate over the still_to_process vertices and add them to the merged
  // graph. Also iterate over their neighbors and, in case they have not been
  // processed yet add them to the still_to_process queue.
  while (!still_to_process_.empty()) {
    const VertexDescriptor source_in_query_graph = still_to_process_.front();
    // Remove the element from the queue.
    still_to_process_.pop();
    LOG(INFO) << "Analyzing " << source_in_query_graph
              << "-th vertex of query graph (index: "
              << query_graph_[source_in_query_graph].index << ").";
    addVertexToMergedGraph(source_in_query_graph);
  }

#ifdef X_VIEW_DEBUG
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  const cv::Mat agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);

  cv::imshow("Similarity", similarity_image);
  cv::imshow("Agreement", agree_similarity_image);

  cv::waitKey();
#endif

  return merged_graph_;
}

void GraphMerger::mergeDuplicates(Graph* graph, const float merge_distance) {

  LOG(INFO) << "Merging all vertices with same semantic label whose euclidean "
            << "distance is smaller than " << merge_distance << ".";

  // Utility class used to store a possible merge between vertices.
  class CandidateMerge {
   public:
    CandidateMerge()
        : anchor_(0), mergeable_(0) {
    }

    CandidateMerge(const VertexDescriptor anchor, const VertexDescriptor mergeable)
        : anchor_(anchor), mergeable_(mergeable) {
      CHECK(anchor < mergeable) << "The anchor vertex descriptor must "
          "have smaller index value than the mergeable!";
    }

    const VertexDescriptor anchor() const { return anchor_; }
    const VertexDescriptor mergeable() const { return mergeable_; }

   private:

    /// \brief Vertex descriptor associated to the 'anchor' vertex, i.e. the
    /// vertex which will absorbe the mergeable_ vertex.
    const VertexDescriptor anchor_;

    /// \brief Vertex descriptor associated to the 'meargeble' vertex, i.e.
    /// the vertex which will be merged into the anchor_ vertex.
    const VertexDescriptor mergeable_;
  };

  // Vector filled up with all possible merges.
  std::vector<CandidateMerge> candidates;

  const uint64_t num_vertices = boost::num_vertices(*graph);

  // This function does not support merges of the type 1-2, 2-3, since after
  // merging 1 with 2, vertex 2 does not exist anymore. For this reason we
  // need to keep track which vertex has already been assigned for a merge.
  std::vector<bool> taken(num_vertices, false);

  LOG(INFO) << "Computing the number of vertex pairs to be merged together.";
  // Traverse the vertices in backward order.
  for (uint64_t i_back = 0; i_back < num_vertices; ++i_back) {
    const uint64_t i = num_vertices - i_back - 1;
    // If the current vertex already wants to be merged to an other, then
    // skip it now.
    if (taken[i] == true)
      continue;

    // Iterate over the remaining elements and create a candidate merge.
    for (uint64_t j_back = 0; j_back < i && taken[i] == false; ++j_back) {
      const uint64_t j = i - j_back - 1;
      // If the current vertex already wants to be merged to an other, then
      // skip it now.
      if (taken[j] == true)
        continue;

      if (GraphMerger::verticesShouldBeMerged(i, j, *graph, merge_distance)) {
        // There is a merge between v_p_i and v_p_j.
        candidates.push_back({j, i});
        taken[i] = taken[j] = true;
      }
    }
  }
  LOG(INFO) << "There are " << candidates.size() << " merging pairs "
            <<"for a graph with " << num_vertices << " vertices in total.";

  // Traverse the candidate merges and merge them
  for(const CandidateMerge& candidate : candidates) {

    const uint64_t anchor = candidate.anchor();
    const uint64_t mergeable = candidate.mergeable();

    // Get a list of all edges going into the mergeable vertex.
    const auto adjacent_vertices =
        boost::adjacent_vertices(mergeable, *graph);

    // Attach the adjacent vertices of mergeable to the anchor.
    for (auto adjacent_vertex = adjacent_vertices.first; adjacent_vertex !=
        adjacent_vertices.second; ++adjacent_vertex) {
      LOG(INFO) << "Adding edge between anchor vertex " << anchor
                << " and neighbor vertex " << *adjacent_vertex
                << " of meargeble vertex " << mergeable << ".";

      // Avoid self-edges (case that happens if anchor was a neighbor of
      // mergeable).
      if(anchor != *adjacent_vertex)
          addEdgeBetweenVertices(anchor, *adjacent_vertex, graph);
    }

    LOG(INFO) << "Updating timestamp of anchor vertex from "
              << (*graph)[anchor].last_time_seen_ << " to "
              << (*graph)[mergeable].last_time_seen_ << ".";
    // Transfer timestamp information to anchor.
    (*graph)[anchor].last_time_seen_ =
        (*graph)[mergeable].last_time_seen_;

    LOG(INFO) << "Removing mergeable vertex from graph structure.";
    // Clear the old mergeable vertex.
    boost::clear_vertex(mergeable, *graph);
    boost::remove_vertex(mergeable, *graph);

  }
}

const bool GraphMerger::verticesShouldBeMerged(const VertexDescriptor v_d_1,
                                               const VertexDescriptor v_d_2,
                                               const Graph& graph,
                                               const float merge_distance) {

  // Query the vertex properties associated to the vertex descriptors passed
  // as argument.
  const VertexProperty& v_p_1 = graph[v_d_1];
  const VertexProperty& v_p_2 = graph[v_d_2];

  // Label consistency: if the semantic label associated to the vertices is
  // different, then the two vertices cannot be merged.
  if(v_p_1.semantic_label != v_p_2.semantic_label)
    return false;

  // Spatial consistency: only merge vertices if their Eucliden distance is
  // smaller than the merge_distance parameter passed as argument.
  const Eigen::Vector3d diff = v_p_1.location_3d - v_p_2.location_3d;
  if(diff.norm() > merge_distance)
    return false;

  // Since all tests are fulfilled, the two vertices should be merged.
  return true;

}

GraphMatcher::MaxSimilarityMatrixType GraphMerger::computeAgreementMatrix() const {

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      matching_result_.computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      matching_result_.computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      (max_similarity_colwise.array() * max_similarity_rowwise.array());

  return max_similarity_agree;
}

void GraphMerger::addVertexToMergedGraph(const VertexDescriptor& source_in_query_graph) {

  // Get correspondent vertex in database graph.
  const VertexDescriptor
      source_in_db_graph = query_in_db_[source_in_query_graph];
  // Get the associated VertexProperty.
  VertexProperty& source_v_p = merged_graph_[source_in_db_graph];
  LOG(INFO) << "\tCorresponds to " << source_in_db_graph
            << "-th vertex in database graph:" << source_v_p << ".";

  const uint64_t new_time_stamp =
      query_graph_[source_in_query_graph].last_time_seen_;
  const uint64_t old_time_stamp =
      source_v_p.last_time_seen_;

  LOG(INFO) << "\tSetting last_time_seen_ property of database vertex from "
            << old_time_stamp << " to " << new_time_stamp << ".";

  source_v_p.last_time_seen_ = new_time_stamp;

  LOG(INFO) << "\tIterating over its "
            << boost::degree(source_in_query_graph, query_graph_)
            << " neighbors in query graph.";

  // Get the list of direct neighbors of the source_in_query_graph.
  const auto neighbors_in_query_graph =
      boost::adjacent_vertices(source_in_query_graph, query_graph_);
  // Loop over the neighbors of query_graph and add them to db_graph if they
  // are not matched.
  int num_neighbor = 0;
  for (auto neighbor_in_query_graph = neighbors_in_query_graph.first;
       neighbor_in_query_graph != neighbors_in_query_graph.second;
       ++neighbor_in_query_graph) {
    LOG(INFO) << "\t\t" << num_neighbor++ << "-th neighbor: "
              << query_graph_[*neighbor_in_query_graph] << ":";
    auto neighbor_pos =
        std::find(matched_vertices_.begin(), matched_vertices_.end(),
                  *neighbor_in_query_graph);
    if (neighbor_pos == matched_vertices_.end()) {
      LOG(INFO) << "\t\tis unmatched, so we simply attach it to the database "
          "graph via the matched vertex " << source_v_p << ".";
      // The neighbor_in_query_graph was unmatched, so we add it to the
      // merged graph.
      VertexProperty neighbor_v_p = query_graph_[*neighbor_in_query_graph];
      // Set the new index.
      neighbor_v_p.index = current_vertex_index_++;
      const VertexDescriptor neighbor_in_db_graph =
          boost::add_vertex(neighbor_v_p, merged_graph_);
      LOG(INFO) << "\t\tWas added to merged graph: " << neighbor_v_p <<
                " graph with VD: " << neighbor_in_db_graph << ".";
      // Create an edge between source_in_db_graph and the newly added vertex.
      auto edge_d = boost::add_edge(source_in_db_graph,
                                    neighbor_in_db_graph,
                                    {source_v_p.index, neighbor_v_p.index},
                                    merged_graph_);
      LOG(INFO) << "\t\tadded corresponding edge "
                << merged_graph_[edge_d.first] << ".";

      CHECK(edge_d.second == true)
      << "Added an unmatched vertex to the merged graph, but edge "
          "between source vertex and neighbor was already existing. "
          "This should not happen";
      // Set the neighbor as matched.
      matched_vertices_.push_back(*neighbor_in_query_graph);
      query_in_db_.insert({*neighbor_in_query_graph, neighbor_in_db_graph});
      still_to_process_.push(*neighbor_in_query_graph);
    } else {
      // Add an edge between the two already merged vertices.
      const VertexDescriptor
          neighbor_in_db_graph = query_in_db_[*neighbor_in_query_graph];
      const VertexProperty neighbor_v_p = merged_graph_[neighbor_in_db_graph];
      LOG(INFO) << "\t\tis already matched, so we simply need to add an edge "
          "between the the two vertices.";
      auto edge_d = boost::add_edge(source_in_db_graph, neighbor_in_db_graph,
                                    {source_v_p.index,
                                     neighbor_v_p.index}, merged_graph_);
      if (edge_d.second == false) {
        LOG(INFO) << "\t\tAn edge was already present in the merged graph...";
      } else {
        LOG(INFO) << "\t\tCreated a new edge in the merged graph even though "
            "the two vertices where already merged.";
      }
    }
  }
}

const uint64_t GraphMerger::temporalDistance(const uint64_t i,
                                             const uint64_t j) const {
  const VertexProperty& v_i_database = database_graph_[i];
  const VertexProperty& v_j_query = query_graph_[j];

  return v_j_query.last_time_seen_ - v_i_database.last_time_seen_;
}


const double GraphMerger::spatialDistance(const uint64_t i, const uint64_t j)
const {
  const VertexProperty& v_i_database = database_graph_[i];
  const VertexProperty& v_j_query = query_graph_[j];

  return (v_j_query.location_3d - v_i_database.location_3d).norm();
}

}

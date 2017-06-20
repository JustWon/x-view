#include <x_view_core/matchers/graph_matcher/graph_merger.h>

#include <x_view_core/matchers/graph_matcher.h>

namespace x_view {

GraphMerger::GraphMerger(const Graph& database_graph,
                         const Graph& query_graph,
                         const GraphMatcher::GraphMatchingResult& matching_result)
    : database_graph_(database_graph),
      query_graph_(query_graph),
      matching_result_(matching_result) {

}

Graph GraphMerger::getMergedGraph() {

  const unsigned long num_query_vertices = boost::num_vertices(query_graph_);
  const unsigned long num_db_vertices = boost::num_vertices(database_graph_);

  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      matching_result_.getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      computeAgreementMatrix();

  // Initialize the merged graph to contain all vertices of the database_graph.
  merged_graph_ = database_graph_;

  const float similarity_threshold = 0.0f;

  // This map contains the matches between the vertex descriptors in the
  // query graph to the corresponding vertex descriptor in the database_graph.
  query_in_db_.clear();

  // Loop over the vertices of the query graph.
  for (int j = 0; j < num_query_vertices; ++j) {
    bool found_match = false;
    // Loop over the vertices of the database graph.
    for (int i = 0; !found_match && i < num_db_vertices; ++i) {
      // Check if this was a possible match.
      if (max_similarity_agree(i, j) == true
          && similarity_matrix(i, j) >= similarity_threshold) {
        // There is a match between the j-th vertex of the query graph and
        // the i-th vertex of the database graph.
        query_in_db_.insert({j, i});
        matched_vertices_.push_back(j);
        found_match = true;
      }
    }
  }

  // Fill the queue of still to process vertices into the queue.
  still_to_process_ = DescriptorQueue();
  for (const VertexDescriptor v_d : matched_vertices_) {
    still_to_process_.push(v_d);
  }

  // Define start vertex index of vertices being added to the merged graph.
  current_vertex_index_ = std::numeric_limits<int>::min();
  const auto db_vertices = boost::vertices(database_graph_);
  for (auto iter = db_vertices.first; iter != db_vertices.second; ++iter)
    current_vertex_index_ =
        std::max(current_vertex_index_, database_graph_[*iter].index_);
  ++current_vertex_index_;


  // Iterate over the still_to_process vertices and add their children to the
  // database graph in a recursive way if they are unmatched.
  while (!still_to_process_.empty()) {
    const VertexDescriptor source_in_query_graph = still_to_process_.front();
    // Remove the element from the queue.
    still_to_process_.pop();
    std::cout << "Analyzing " << source_in_query_graph
              << "-th vertex of query graph" << std::endl;
    addVertexToMergedGraph(source_in_query_graph);
  }

  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  const cv::Mat agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);

  cv::imshow("Similarity", similarity_image);
  cv::imshow("Agreement", agree_similarity_image);

  cv::waitKey();

  return merged_graph_;

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
  const VertexProperty source_v_p = merged_graph_[source_in_db_graph];
  std::cout << "\tCorresponds to " << source_in_db_graph
            << "-th vertex in g1:"
            << source_v_p << std::endl;

  std::cout << "\tIterating over " << boost::degree(source_in_query_graph,
                                                    query_graph_)
            << " neighbors in g2" << std::endl;
  // Get the list of direct neighbors of the source_in_query_graph.
  const auto neighbors_in_query_graph =
      boost::adjacent_vertices(source_in_query_graph, query_graph_);
  // Loop over the neighbors of query_graph and add them to db_graph if they
  // are not matched.
  int num_neighbor = 0;
  for (auto neighbor_in_query_graph = neighbors_in_query_graph.first;
       neighbor_in_query_graph != neighbors_in_query_graph.second;
       ++neighbor_in_query_graph) {
    std::cout << "\t\t" << num_neighbor++ << "-th neighbor: "
              << query_graph_[*neighbor_in_query_graph];
    auto neighbor_pos =
        std::find(matched_vertices_.begin(), matched_vertices_.end(),
                  *neighbor_in_query_graph);
    if (neighbor_pos == matched_vertices_.end()) {
      std::cout << " is unmatched" << std::endl;
      // The neighbor_in_query_graph was unmatched, so we add it to the
      // merged graph.
      VertexProperty neighbor_v_p = query_graph_[*neighbor_in_query_graph];
      // Set the new index.
      neighbor_v_p.index_ = current_vertex_index_++;
      const VertexDescriptor neighbor_in_db_graph =
          boost::add_vertex(neighbor_v_p, merged_graph_);
      std::cout << "\t\tWas added to merged graph: " << neighbor_v_p <<
                " graph with VD: " << neighbor_in_db_graph << std::endl;
      // Create an edge between source_in_g1 and the newly added vertex.
      auto edge_d = boost::add_edge(source_in_db_graph,
                                    neighbor_in_db_graph,
                                    {source_v_p.index_, neighbor_v_p.index_},
                                    merged_graph_);
      std::cout << "\t\tadded corresponding edge "
                << merged_graph_[edge_d.first] << std::endl;

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
      std::cout << " is already matched" << std::endl;
      auto edge_d = boost::add_edge(source_in_db_graph, neighbor_in_db_graph,
                                    {source_v_p.index_,
                                     neighbor_v_p.index_}, merged_graph_);
      if (edge_d.second == false) {
        std::cout << "\t\tAn edge was already present in the merged graph..."
                  << std::endl;
      } else {
        std::cout << "\t\tCreated a new edge in the merged graph even though "
            "the two vertices where already merged." << std::endl;
      }
    }
  }
}


}
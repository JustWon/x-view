#include "test_graph_merger.h"

#include <x_view_core/matchers/graph_matcher.h>

#include <queue>

namespace x_view_test {

void mergeGraphs(const Graph& g1, const Graph g2, Graph* merged) {

  CHECK_NOTNULL(merged);

  // Define the parameters needed to match the graphs.
  RandomWalkerParams random_walker_params;
  random_walker_params.num_walks_ = 1000;
  random_walker_params.walk_length_ = 3;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;

  VertexSimilarity::SCORE_TYPE score_type = VertexSimilarity::SCORE_TYPE::HARD;

  // Compute the similarities between the graphs via the GraphMatcher.
  GraphMatcher graph_matcher(random_walker_params, score_type);

  // Add the first graph to the matcher.
  graph_matcher.addDescriptor(g1);

  // Match the second graph to the first one and get the matching result.
  const auto matching_result =
      std::dynamic_pointer_cast<GraphMatcher::GraphMatchingResult>(
          graph_matcher.match(g2));

  // Verify that the cast was successful
  CHECK_NOTNULL(matching_result.get());

  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      matching_result->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      matching_result->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      matching_result->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      (max_similarity_colwise.array() * max_similarity_rowwise.array());

  // Initialize the merged graph to contain all vertices of g1.
  *merged = g1;

  const float similarity_threshold = 0.0f;

  // This map contains the matches between the vertex descriptors in g2 to
  // the corresponding vertex descriptor in g1.
  std::unordered_map<VertexDescriptor, VertexDescriptor> g2_in_g1;

  // List of vertices in g2 that have correspondence in g1.
  std::vector<VertexDescriptor> matched_vertices;

  for (int j = 0; j < similarity_matrix.cols(); ++j) {
    const VertexProperty v_p_j = g2[j];
    bool found_match = false;
    for (int i = 0; !found_match && i < similarity_matrix.rows(); ++i) {
      const VertexProperty v_p_i = g1[i];
      // Check if this was a possible match.
      if (max_similarity_agree(i, j) == true
          && similarity_matrix(i, j) >= similarity_threshold) {
        // There is a match between the j-th vertex of the query graph g2 and
        // the i-th vertex of the database graph g1.
        g2_in_g1.insert({j, i});
        matched_vertices.push_back(j);
        found_match = true;
      }
    }
  }

  // Fill the queue of still to process vertices.
  std::queue<VertexDescriptor> still_to_process;
  for (const VertexDescriptor v_d : matched_vertices) {
    still_to_process.push(v_d);
  }

  // Define start vertex index of newly added vertices to the merged graph.
  int current_vertex_index = std::numeric_limits<int>::min();
  const auto g1_vertices = boost::vertices(g1);
  for (auto iter = g1_vertices.first; iter != g1_vertices.second; ++iter)
    current_vertex_index = std::max(current_vertex_index, g1[*iter].index_);
  ++current_vertex_index;

  // Lambda function used to traverse the graph tree to add unmatched
  // vertices to the database graph. This function assumes that
  // the passed argument has already been added to the database graph.
  auto traverseAndAdd = [&](const VertexDescriptor& source_in_g2_v_d) {
    // Get correspondent vertex in g1.
    const VertexDescriptor source_in_g1_v_d = g2_in_g1[source_in_g2_v_d];
    // Get the associated VertexProperty.
    const VertexProperty source_v_p = (*merged)[source_in_g1_v_d];
    std::cout << "\tCorresponds to " << source_in_g1_v_d << "-th vertex in g1:"
              << source_v_p << std::endl;

    std::cout << "\tIterating over " << boost::degree(source_in_g2_v_d, g2)
              << " neighbors in g2" << std::endl;
    // Get the list of direct neighbors of the source_in_g2.
    const auto neighbors_in_g2 = boost::adjacent_vertices(source_in_g2_v_d, g2);
    // Loop over the neighbors of g2 and add them to g1 if they are not matched.
    int num_neighbor = 0;
    for (auto neighbor = neighbors_in_g2.first;
         neighbor != neighbors_in_g2.second; ++neighbor) {
      std::cout << "\t\t" << num_neighbor++ << "-th neighbor: "
                << g2[*neighbor];
      auto neighbor_pos = std::find(matched_vertices.begin(),
                                    matched_vertices.end(), *neighbor);
      if (neighbor_pos == matched_vertices.end()) {
        std::cout << " is unmatched" << std::endl;
        // The neighbor was unmatched, so we add it to the merged graph.
        VertexProperty neighbor_v_p = g2[*neighbor];
        // Set the new index.
        neighbor_v_p.index_ = current_vertex_index++;
        const VertexDescriptor neighbor_in_g1_v_d =
            boost::add_vertex(neighbor_v_p, *merged);
        std::cout << "\t\tWas added to merged graph: " << neighbor_v_p <<
                  " graph with VD: " << neighbor_in_g1_v_d << std::endl;
        // Create an edge between source_in_g1 and the newly added vertex.
        auto edge_d = boost::add_edge(source_in_g1_v_d,
                                      neighbor_in_g1_v_d,
                                      {source_v_p.index_, neighbor_v_p.index_},
                                      *merged);
        std::cout << "\t\tadded corresponding edge "
                  << (*merged)[edge_d.first] << std::endl;

        CHECK(edge_d.second == true)
        << "Added an unmatched vertex to the merged graph, but edge "
            "between source vertex and neighbor was already existing. "
            "This should not happen";
        // Set the neighbor as matched.
        matched_vertices.push_back(*neighbor);
        g2_in_g1.insert({*neighbor, neighbor_in_g1_v_d});
        still_to_process.push(*neighbor);
      } else {
        // Add an edge between the two already merged vertices.
        const VertexDescriptor neighbor_in_g1_v_d = g2_in_g1[*neighbor];
        const VertexProperty neighbor_in_g1_v_p = (*merged)[neighbor_in_g1_v_d];
        std::cout << " is already matched" << std::endl;
        auto edge_d = boost::add_edge(source_in_g1_v_d, neighbor_in_g1_v_d,
                                      {source_v_p.index_,
                                       neighbor_in_g1_v_p.index_}, *merged);
        if (edge_d.second == false) {
          std::cout << "\t\tAn edge was already present in the merged graph..."
                    << std::endl;
        } else {
          std::cout << "\t\tCreated a new edge in the merged graph even though "
              "the two vertices where already merged." << std::endl;
        }
      }
    }
  };

  // Iterate over the matched vertices and add their children to the database
  // graph in a recursive way if they are unmatched.
  while (!still_to_process.empty()) {
    const VertexDescriptor source_in_g2 = still_to_process.front();
    still_to_process.pop();
    std::cout << "Analyzing " << source_in_g2 << "-th vertex of g2" <<
              std::endl;
    traverseAndAdd(source_in_g2);
  }

  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  const cv::Mat agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);

  cv::imshow("Similarity", similarity_image);
  cv::imshow("Agreement", agree_similarity_image);

  cv::waitKey();

}

}
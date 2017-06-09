#include <x_view_core/matchers/graph_matcher/graph_matcher.h>

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/similarity_plotter.h>

#include <boost/graph/copy.hpp>

namespace x_view {

GraphMatcher::GraphMatcher() {

  // Set up parameters for matchings.
  VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

}

GraphMatcher::~GraphMatcher() {
}

AbstractMatcher::MatchingResultPtr GraphMatcher::match(const SemanticLandmarkPtr& query_landmark) {

  std::cout << "Beginning the matching" << std::endl;

  // Cast the SemanticLandmarkPtr to a GraphLandmarkPtr.
  const auto graph_landmark_ptr =
      std::dynamic_pointer_cast<const GraphLandmark>(query_landmark);

  CHECK(graph_landmark_ptr != nullptr) << "Impossible to cast 'query_landmark' "
      "to a 'GraphLandmark' pointer. Be sure the passed 'SemanticLandmarkPtr' "
      "points to an instance of 'GraphLandmark'.";

  // Cast the descriptor associated to the graph_landmark_ptr to a
  // GraphDescriptor.
  const auto graph_descriptor =
      std::dynamic_pointer_cast<const GraphDescriptor>(
          graph_landmark_ptr->getDescriptor());

  // Perform checks related to the cast.
  CHECK(graph_descriptor != nullptr) << "Impossible to cast descriptor "
      "associated to graph_landmark_ptr to a 'const GraphDescriptor'";

  // Extract the representation of the descriptor from the graph_descriptor
  // object.
  const Graph& query_semantic_graph = graph_descriptor->getDescriptor();

  // Extract the random walks of the graph.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = 3;
  random_walker_params.num_walks_ = 100;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM;

  RandomWalker random_walker(query_semantic_graph, random_walker_params);
  random_walker.generateRandomWalks();

  // If we don't have any vertices in the global_semantic_graph_, initialize it
  // with the query_semantic_graph (this happens the first time the graph
  // matcher is used).
  if (boost::num_vertices(global_semantic_graph_) == 0) {

    global_semantic_graph_.clear();
    global_semantic_graph_ = query_semantic_graph;

    global_walk_map_vector_.clear();
    global_walk_map_vector_ = random_walker.getMappedWalks();

    // Return empty GraphMatchingResult as no matches have been computed.
    return std::make_shared<GraphMatchingResult>();
  } else {

    // Create a matching result pointer which will be returned by this
    // function which stores the similarity matrix.
    auto matchingResult = std::make_shared<GraphMatchingResult>();

    Eigen::MatrixXf& similarity_matrix = matchingResult->getSimilarityMatrix();

    const VertexSimilarity::SCORE_TYPE score_type =
        VertexSimilarity::SCORE_TYPE::HARD;
    computeSimilarityMatrix(random_walker, &similarity_matrix, score_type);

    // Display the computed similarities.
    const cv::Mat similarity_image =
        SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
    cv::imshow("Vertex similarity", similarity_image);

    const cv::Mat max_col_similarity_image =
        SimilarityPlotter::getMaxColwiseImageFromSimilarityMatrix(
            similarity_matrix);
    cv::imshow("Max col vertex similarity", max_col_similarity_image);

    const cv::Mat max_row_similarity_image =
        SimilarityPlotter::getMaxRowwiseImageFromSimilarityMatrix(
            similarity_matrix);
    cv::imshow("Max row vertex similarity", max_row_similarity_image);

    cv::waitKey(200);

    const auto query_vertices = boost::vertices(query_semantic_graph);
    for (auto vertex_iter = query_vertices.first; vertex_iter !=
        query_vertices.second; ++vertex_iter) {
      const VertexProperty v_p = query_semantic_graph[*vertex_iter];
      boost::add_vertex(v_p, global_semantic_graph_);
    }

    const auto query_edges = boost::edges(query_semantic_graph);
    const unsigned long num_global_vertices =
        boost::num_vertices(global_semantic_graph_);

    for (auto edge_iter = query_edges.first; edge_iter != query_edges.second;
         ++edge_iter) {
      const VertexDescriptor v_d_source =
          boost::source(*edge_iter, query_semantic_graph);
      const VertexDescriptor v_d_target =
          boost::target(*edge_iter, query_semantic_graph);

      const VertexProperty& v_p_source = query_semantic_graph[v_d_source];
      const VertexProperty& v_p_target = query_semantic_graph[v_d_target];

      EdgeProperty e_p;
      e_p.from_ = v_p_source.index_;
      e_p.to_ = v_p_target.index_;
      boost::add_edge(v_d_source + num_global_vertices,
                      v_d_target + num_global_vertices,
                      e_p, global_semantic_graph_);
    }

    // Extract the random walks from the RandomWalker passed as argument.
    const std::vector<RandomWalker::WalkMap>& query_walk_map_vector =
        random_walker.getMappedWalks();

    // Add the query random walks to the global map.
    global_walk_map_vector_.insert(global_walk_map_vector_.end(),
                                   query_walk_map_vector.begin(),
                                   query_walk_map_vector.end());



    // Return the matching result filled with the matches.
    return matchingResult;
  }
}

LandmarksMatcherPtr GraphMatcher::create() {
  return std::make_shared<GraphMatcher>();
}

void GraphMatcher::computeSimilarityMatrix(const RandomWalker& random_walker,
                                           Eigen::MatrixXf* similarity_matrix,
                                           const VertexSimilarity::SCORE_TYPE score_type) {

  CHECK_NOTNULL(similarity_matrix);

  // Extract the random walks from the RandomWalker passed as argument.
  const std::vector<RandomWalker::WalkMap>& query_walk_map_vector =
      random_walker.getMappedWalks();

  // Extract the graph over which the random walks are defined.
  const Graph& query_graph = random_walker.graph();

  const unsigned long num_global_vertices =
      boost::num_vertices(global_semantic_graph_);
  const unsigned long num_query_vertices =
      boost::num_vertices(query_graph);
  similarity_matrix->resize(num_global_vertices, num_query_vertices);

  // Set the similarity score type to be used.
  VertexSimilarity::setScoreType(score_type);

  // Fill up similarity matrix.
  for (int i = 0; i < num_global_vertices; ++i) {
    const auto& vertex_d_i = boost::vertex(i, global_semantic_graph_);
    const auto& vertex_p_i = global_semantic_graph_[vertex_d_i];
    const auto& mapped_walks_i = global_walk_map_vector_[i];
    for (int j = 0; j < num_query_vertices; ++j) {
      const auto& vertex_d_j = boost::vertex(j, query_graph);
      const auto& vertex_p_j = query_graph[vertex_d_j];
      // Score is only nonzero if the source vertex has same semantic label.
      if (vertex_p_i.semantic_label_ == vertex_p_j.semantic_label_) {
        const auto& mapped_walks_j = query_walk_map_vector[j];
        const float similarity =
            VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
        similarity_matrix->operator()(i, j) = similarity;
      }
    }
  }

}

}


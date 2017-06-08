#include <x_view_core/matchers/graph_matcher/graph_matcher.h>

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

#include <boost/graph/copy.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>


namespace x_view {

GraphMatcher::GraphMatcher() {

  // Set up parameters for matchings.
  VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

}

GraphMatcher::~GraphMatcher() {
}

AbstractMatcher::MatchingResultPtr GraphMatcher::match(const SemanticLandmarkPtr& query_landmark) {

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

  const std::vector<RandomWalker::WalkMap>& query_walk_map_vector =
      random_walker.getMappedWalks();

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
    const unsigned long num_global_vertices =
        boost::num_vertices(global_semantic_graph_);
    const unsigned long num_query_vertices =
        boost::num_vertices(query_semantic_graph);
    Eigen::MatrixXf similarity_matrix(num_global_vertices, num_query_vertices);

    // Set the similarity score type to be used.
    VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

    // Fill up similarity matrix.
    for (int i = 0; i < num_global_vertices; ++i) {
      const auto& vertex_d_i = boost::vertex(i, global_semantic_graph_);
      const auto& vertex_p_i = global_semantic_graph_[vertex_d_i];
      const auto& mapped_walks_i = global_walk_map_vector_[i];
      for (int j = 0; j < num_query_vertices; ++j) {
        const auto& vertex_d_j = boost::vertex(j, query_semantic_graph);
        const auto& vertex_p_j = query_semantic_graph[vertex_d_j];
        // Score is only nonzero if the source vertex has same semantic label.
        if (vertex_p_i.semantic_label_ == vertex_p_j.semantic_label_) {
          const auto& mapped_walks_j = query_walk_map_vector[j];
          const float similarity =
              VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
          similarity_matrix(i, j) = similarity;
        }
      }
    }

    auto roundToClosestMultiple = [](const int number, const int multiple) {
      return ((number + multiple / 2) / multiple) * multiple;
    };
    const int desired_size = 400;
    const int resulting_rows =
        roundToClosestMultiple(desired_size, similarity_matrix.rows());
    const int resulting_cols =
        roundToClosestMultiple(desired_size, similarity_matrix.cols());

    // Normalize score matrix and convert them to opencv image.
    const float max_score = similarity_matrix.maxCoeff();
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
    normalized_similarity =
        (similarity_matrix / max_score * 255).cast <unsigned char>();
    cv::Mat cv_scores, color_scores;
    cv::eigen2cv(normalized_similarity, cv_scores);

    cv::resize(cv_scores, cv_scores, cv::Size(resulting_cols, resulting_rows),
               0, 0, cv::INTER_NEAREST);

    cv::applyColorMap(cv_scores, color_scores, cv::COLORMAP_OCEAN);

    cv::imshow("Similarity score ", color_scores);

    const auto query_vertices = boost::vertices(query_semantic_graph);
    for(auto vertex_iter = query_vertices.first; vertex_iter !=
        query_vertices.second; ++ vertex_iter) {
      const VertexProperty v_p = query_semantic_graph[*vertex_iter];
      boost::add_vertex(v_p, global_semantic_graph_);
    }

    const auto query_edges = boost::edges(query_semantic_graph);
    for(auto edge_iter = query_edges.first; edge_iter != query_edges.second;
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
                      e_p,                      global_semantic_graph_);
    }

    // Add the query random walks to the global map.
    global_walk_map_vector_.insert(global_walk_map_vector_.end(),
                                   query_walk_map_vector.begin(),
                                   query_walk_map_vector.end());

    // Create a matching result pointer which will be returned by this function.
    auto matchingResult =
        std::make_shared<GraphMatchingResult>(GraphMatchingResult(similarity_matrix));

    // Return the matching result filled with the matches.
    return matchingResult;
  }
}

LandmarksMatcherPtr GraphMatcher::create() {
  return std::make_shared<GraphMatcher>();
}

}


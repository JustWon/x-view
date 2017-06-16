#include "test_graph_landmark_matcher.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/matchers/graph_matcher.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>

namespace x_view_test {

typedef std::shared_ptr<GraphMatcher> GraphMatcherPtr;

#define CAST(from, to) std::dynamic_pointer_cast<to>(from)

void testChainGraph(const unsigned long seed) {


  // Define parameters for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices_ = 50;
  construction_params.num_semantic_classes_
      = global_dataset_ptr->numSemanticClasses();
  construction_params.seed_ = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add_ = 0;
  modifier_params.num_links_for_new_vertices_ = 0;
  modifier_params.num_vertices_to_remove_ = 0;
  modifier_params.num_edges_to_add_ = 0;
  modifier_params.num_edges_to_remove_ = 0;

  // Size of the extracted subgraph.
  const int extraction_radius = 6;

  GraphPair graph_pair_chain = generateChainGraphPair(construction_params,
                                                      modifier_params,
                                                      extraction_radius);

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = 3;
  random_walker_params.num_walks_ = 200;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;

  GraphMatcherPtr graph_matcher_ptr =
      CAST(GraphMatcher::create(random_walker_params,
                                VertexSimilarity::SCORE_TYPE::HARD),
           GraphMatcher);

  CHECK_NOTNULL(graph_matcher_ptr.get());

  // Add the base graph to the matcher.
  graph_matcher_ptr->addDescriptor(graph_pair_chain.base_graph_);

  // Match the subgraph to the entire graph.
  auto matching_result = graph_matcher_ptr->match(graph_pair_chain.sub_graph_);

  const float accuracy = similarityAccuracy(graph_pair_chain, matching_result);
  std::cout << "Chain matching has accuracy of " << accuracy << std::endl;

#ifdef X_VIEW_DEBUG
  // Compute similarity matrices.
  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  // Display the computed similarities.
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  cv::imshow("Vertex similarity", similarity_image);

  const cv::Mat max_col_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_colwise);
  cv::imshow("Max col similarity", max_col_similarity_image);

  const cv::Mat max_row_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_rowwise);
  cv::imshow("Max row similarity", max_row_similarity_image);

  const cv::Mat max_agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);
  cv::imshow("Max agree similarity", max_agree_similarity_image);

  cv::waitKey();
#endif

}

void testRandomGraph(const unsigned long seed) {


  // Define parameter for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices_ = 500;
  construction_params.edge_probability_ = 0.001;
  construction_params.num_semantic_classes_
      = global_dataset_ptr->numSemanticClasses();
  construction_params.seed_ = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add_ = 50;
  modifier_params.num_links_for_new_vertices_ = 2;
  modifier_params.num_vertices_to_remove_ = 50;
  modifier_params.num_edges_to_add_ = 20;
  modifier_params.num_edges_to_remove_ = 20;

  const int extraction_radius = 3;

  GraphPair graph_pair_random = generateRandomGraphPair(construction_params,
                                                        modifier_params,
                                                        extraction_radius);

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = 3;
  random_walker_params.num_walks_ = 200;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;

  GraphMatcherPtr graph_matcher_ptr =
      CAST(GraphMatcher::create(random_walker_params,
                                VertexSimilarity::SCORE_TYPE::HARD),
           GraphMatcher);

  CHECK_NOTNULL(graph_matcher_ptr.get());

  // Add the base graph to the matcher.
  graph_matcher_ptr->addDescriptor(graph_pair_random.base_graph_);

  // Match the subgraph to the entire graph.
  auto matching_result = graph_matcher_ptr->match(graph_pair_random.sub_graph_);

  const float accuracy =
      similarityAccuracy(graph_pair_random, matching_result);

  std::cout << "Random matching has accuracy of " << accuracy << std::endl;

#ifdef X_VIEW_DEBUG
  // Compute similarity matrices.
  const GraphMatcher::SimilarityMatrixType& similarity_matrix =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->getSimilarityMatrix();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      std::dynamic_pointer_cast<const GraphMatcher::GraphMatchingResult>
          (matching_result)->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);

  // Display the computed similarities.
  const cv::Mat similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(similarity_matrix);
  cv::imshow("Vertex similarity", similarity_image);

  const cv::Mat max_col_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_colwise);
  cv::imshow("Max col similarity", max_col_similarity_image);

  const cv::Mat max_row_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_rowwise);
  cv::imshow("Max row similarity", max_row_similarity_image);

  const cv::Mat max_agree_similarity_image =
      SimilarityPlotter::getImageFromSimilarityMatrix(max_similarity_agree);
  cv::imshow("Max agree similarity", max_agree_similarity_image);

  cv::waitKey();
#endif
}

GraphPair generateChainGraphPair(const GraphConstructionParams& construction_params,
                                 const GraphModifierParams& modifier_params,
                                 const int extraction_radius) {

  // Seed randomization.
  std::mt19937 rng(construction_params.seed_);

  GraphPair graph_pair;
  // Generate a chain graph with the parameters specified above.
  graph_pair.base_graph_ = generateChainGraph(construction_params);

  const VertexDescriptor source_vertex =
      boost::random_vertex(graph_pair.base_graph_, rng);
  graph_pair.sub_graph_ =
      extractSubgraphAroundVertex(graph_pair.base_graph_, source_vertex,
                                  extraction_radius);

  // Effectively add and remove vertices and edges from the graph.
  modifyGraph(&graph_pair.sub_graph_, modifier_params, rng);

  LOG(INFO) << "Generated chain graph with "
            << boost::num_vertices(graph_pair.base_graph_)
            << " vertices and extracted a subgraph with "
            << boost::num_vertices(graph_pair.sub_graph_) << " vertices";

  return graph_pair;
}

GraphPair generateRandomGraphPair(const GraphConstructionParams& construction_params,
                                  const GraphModifierParams& modifier_params,
                                  const int extraction_radius) {

  // Seed randomization.
  std::mt19937 rng(construction_params.seed_);

  GraphPair graph_pair;
  // Generate a random graph with the parameters specified above.
  graph_pair.base_graph_ = generateRandomGraph(construction_params);

  const VertexDescriptor source_vertex =
      boost::random_vertex(graph_pair.base_graph_, rng);
  graph_pair.sub_graph_ =
      extractSubgraphAroundVertex(graph_pair.base_graph_, source_vertex,
                                  extraction_radius);
  // Effectively add and remove vertices and edges from the graph.
  modifyGraph(&graph_pair.sub_graph_, modifier_params, rng);

  LOG(INFO) << "Generated random graph with "
            << boost::num_vertices(graph_pair.base_graph_)
            << " vertices and extracted a subgraph with "
            << boost::num_vertices(graph_pair.sub_graph_) << " vertices";

  return graph_pair;
}

float similarityAccuracy(const GraphPair& graph_pair,
                         const AbstractMatcher::MatchingResultPtr&
                         matching_result_ptr) {
  const Graph& base_graph = graph_pair.base_graph_;
  const Graph& sub_graph = graph_pair.sub_graph_;

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_colwise =
      CAST(matching_result_ptr, GraphMatcher::GraphMatchingResult)
          ->computeMaxSimilarityColwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_rowwise =
      CAST(matching_result_ptr, GraphMatcher::GraphMatchingResult)
          ->computeMaxSimilarityRowwise();

  const GraphMatcher::MaxSimilarityMatrixType max_similarity_agree =
      max_similarity_colwise.cwiseProduct(max_similarity_rowwise);


  int correct_matches = 0;
  int num_proposed_matches = 0;
  // Loop over agree similarity matrix and check if the agreed match is a true
  // match.
  for (int i = 0; i < max_similarity_agree.rows(); ++i) {
    for (int j = 0; j < max_similarity_agree.cols(); ++j) {
      if (max_similarity_agree(i, j) == true) {
        // Retrieve the indices of the i-th and j-th vertices of the
        // base_graph_ and sub_graph_ respectively.
        const VertexProperty v_i_base = base_graph[i];
        const VertexProperty v_j_sub = sub_graph[j];

        if (v_i_base.index_ == v_j_sub.index_)
          ++correct_matches;

        ++num_proposed_matches;

      }
    }
  }

  return static_cast<float>(correct_matches) / num_proposed_matches;

}

void modifyGraph(Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  std::vector<int> component(boost::num_vertices(*graph));
  int num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "Input graph to " << __FUNCTION__ << " has " << num_connected_components
  << " connected components. Function " << __FUNCTION__
  << " only works with single components.";

  for (int i = 0; i < params.num_vertices_to_add_; ++i)
    addRandomVertexToGraph(graph, rng, params.num_links_for_new_vertices_);

  for (int i = 0; i < params.num_edges_to_add_; ++i)
    addRandomEdgeToGraph(graph, rng);

  for (int i = 0; i < params.num_vertices_to_remove_; ++i)
    removeRandomVertexFromGraph(graph, rng);

  for (int i = 0; i < params.num_edges_to_remove_; ++i) {
    removeRandomEdgeFromGraph(graph, rng);
  }

  component.resize(boost::num_vertices(*graph));
  num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "After removing and adding vertices/edges to the graph, there are "
  << "disconnected  components.";

}

}


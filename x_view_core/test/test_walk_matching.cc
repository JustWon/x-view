#include "test_walk_matching.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace x_view_test {

void testChainGraph(const unsigned long seed) {
  // Graph construction parameters
  const int num_vertices = 50;

  // Graph modifier parameters
  const int num_vertices_to_add = 0;
  const int num_links_for_new_vertices = 0;
  const int num_vertices_to_remove = 0;
  const int num_edges_to_add = 2;
  const int num_edges_to_remove = 0;

  // Size of the extracted subgraph.
  const int extraction_radius = 4;

  // Random walker params
  const int num_walks = 200;
  const int walk_length = 3;
  const RandomWalkerParams::RANDOM_SAMPLING_TYPE sampling_type =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  const bool force_visiting_each_neighbor = true;
  const bool allow_returning_back = true;

  // Define parameter for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices_ = num_vertices;
  construction_params.num_semantic_classes_
      = global_dataset_ptr->numSemanticClasses();
  construction_params.seed_ = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add_ = num_vertices_to_add;
  modifier_params.num_links_for_new_vertices_ = num_links_for_new_vertices;
  modifier_params.num_vertices_to_remove_ = num_vertices_to_remove;
  modifier_params.num_edges_to_add_ = num_edges_to_add;
  modifier_params.num_edges_to_remove_ = num_edges_to_remove;

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = walk_length;
  random_walker_params.num_walks_ = num_walks;
  random_walker_params.random_sampling_type_ = sampling_type;
  random_walker_params.force_visiting_each_neighbor_
      = force_visiting_each_neighbor;
  random_walker_params.allow_returning_back_ = allow_returning_back;


  GraphPair graph_pair_chain = generateChainGraphPair(construction_params,
                                                      modifier_params,
                                                      extraction_radius);
  Eigen::MatrixXf chain_similarity =
      computeVertexSimilarity(graph_pair_chain, random_walker_params);

  const int desired_size = 500;
  displaySimilarityMatrix(chain_similarity, desired_size, "chain");
}

void testRandomGraph(const unsigned long seed) {
  // Graph construction parameters
  const int num_vertices = 500;
  const float edge_probability = 0.01;

  // Graph modifier parameters
  const int num_vertices_to_add = 10;
  const int num_links_for_new_vertices = 3;
  const int num_vertices_to_remove = 10;
  const int num_edges_to_add = 10;
  const int num_edges_to_remove = 20;

  // Size of the extracted subgraph.
  const int extraction_radius = 2;

  // Random walker params
  const int num_walks = 200;
  const int walk_length = 3;
  const RandomWalkerParams::RANDOM_SAMPLING_TYPE sampling_type =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  const bool force_visiting_each_neighbor = true;
  const bool allow_returning_back = true;

  // Define parameter for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices_ = num_vertices;
  construction_params.edge_probability_ = edge_probability;
  construction_params.num_semantic_classes_
      = global_dataset_ptr->numSemanticClasses();
  construction_params.seed_ = seed;

  // Define parameters for modifying the graphs.
  GraphModifierParams modifier_params;
  modifier_params.num_vertices_to_add_ = num_vertices_to_add;
  modifier_params.num_links_for_new_vertices_ = num_links_for_new_vertices;
  modifier_params.num_vertices_to_remove_ = num_vertices_to_remove;
  modifier_params.num_edges_to_add_ = num_edges_to_add;
  modifier_params.num_edges_to_remove_ = num_edges_to_remove;

  // Define parameters for random walk extraction.
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = walk_length;
  random_walker_params.num_walks_ = num_walks;
  random_walker_params.random_sampling_type_ = sampling_type;
  random_walker_params.force_visiting_each_neighbor_
      = force_visiting_each_neighbor;
  random_walker_params.allow_returning_back_ = allow_returning_back;

  GraphPair graph_pair_random = generateRandomGraphPair(construction_params,
                                                        modifier_params,
                                                        extraction_radius);
  Eigen::MatrixXf random_similarity =
      computeVertexSimilarity(graph_pair_random, random_walker_params);

  const int desired_size = 500;
  displaySimilarityMatrix(random_similarity, desired_size, "random");
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



Eigen::MatrixXf computeVertexSimilarity(const GraphPair& graph_pair,
                                        const RandomWalkerParams& random_walker_params) {

  const Graph& base_graph = graph_pair.base_graph_;
  RandomWalker random_walker_base(base_graph, random_walker_params);
  random_walker_base.generateRandomWalks();
  const unsigned long num_vertices_base = boost::num_vertices(base_graph);

  const Graph& sub_graph = graph_pair.sub_graph_;
  RandomWalker random_walker_sub(sub_graph, random_walker_params);
  random_walker_sub.generateRandomWalks();
  const unsigned long num_vertices_sub = boost::num_vertices(sub_graph);

  Eigen::MatrixXf scores(num_vertices_base, num_vertices_sub);
  scores.setZero();

  const auto& mapped_walks1 = random_walker_base.getMappedWalks();
  const auto& mapped_walks2 = random_walker_sub.getMappedWalks();

  // Set the similarity score type to be used.
  VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

  // Fill up similarity matrix.
  for (int i = 0; i < num_vertices_base; ++i) {
    const auto& vertex_d_i = boost::vertex(i, base_graph);
    const auto& vertex_p_i = base_graph[vertex_d_i];
    const auto& mapped_walks_i = mapped_walks1[i];
    for (int j = 0; j < num_vertices_sub; ++j) {
      const auto& vertex_d_j = boost::vertex(j, sub_graph);
      const auto& vertex_p_j = sub_graph[vertex_d_j];
      // Score is only nonzero if the source vertex has same semantic label.
      if (vertex_p_i.semantic_label_ == vertex_p_j.semantic_label_) {
        const auto& mapped_walks_j = mapped_walks2[j];
        const float score =
            VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
        scores(i, j) = score;
      }
    }
  }

  return scores;
}

void displaySimilarityMatrix(const Eigen::MatrixXf& similarity_matrix,
                             const int desired_size,
                             const std::string& name) {

  auto roundToClosestMultiple = [](const int number, const int multiple) {
    return ((number + multiple / 2) / multiple) * multiple;
  };
  const int resulting_rows =
      roundToClosestMultiple(desired_size, similarity_matrix.rows());
  const int resulting_cols =
      roundToClosestMultiple(desired_size, similarity_matrix.cols());

  // Normalize score matrix and convert them to opencv image.
  const float max_score = similarity_matrix.maxCoeff();
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
  normalized_similarity =  (similarity_matrix / max_score * 255).cast<unsigned char>();
  cv::Mat cv_scores, color_scores;
  cv::eigen2cv(normalized_similarity, cv_scores);

  cv::resize(cv_scores, cv_scores, cv::Size(resulting_cols, resulting_rows),
             0, 0, cv::INTER_NEAREST);

  cv::applyColorMap(cv_scores, color_scores, cv::COLORMAP_BONE);

  cv::imshow("Similarity score " + name, color_scores);

  // Only show matches which achieve score larger than score_filter.
  const float score_filter = 0.f;
  Eigen::MatrixXf
      scores_max(similarity_matrix.rows(), similarity_matrix.cols());
  scores_max.setZero();
  for (int i = 0; i < similarity_matrix.cols(); ++i) {
    int max_index;
    if (similarity_matrix.col(i).maxCoeff(&max_index) > score_filter)
      scores_max(max_index, i) = 1;
  }
  cv::Mat show_scores_max;
  cv::eigen2cv(scores_max, show_scores_max);
  cv::resize(show_scores_max, show_scores_max,
             cv::Size(resulting_cols, resulting_rows),
             0, 0, cv::INTER_NEAREST);

  cv::imshow("Max scores " + name, show_scores_max);
  cv::waitKey();
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


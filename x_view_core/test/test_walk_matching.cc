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

  for (int i = 0; i < params.num_vertices_to_remove_; ++i)
    removeRandomVertexFromGraph(graph, rng);

  for (int i = 0; i < params.num_edges_to_add_; ++i)
    addRandomEdgeToGraph(graph, rng);

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

void computeVertexSimilarity(const GraphPair& graph_pair,
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

  // Display the generated similarities as a heatmap image.
  const int desired_heatmap_size = 800;
  const float reshape_factor =
      static_cast<float>(desired_heatmap_size)
          / std::max(scores.cols(), scores.rows());

  // Normalize score matrix and convert it to opencv image.
  const float max_score = scores.maxCoeff();
  scores /= max_score;
  cv::Mat show_scores;
  cv::eigen2cv(scores, show_scores);

  cv::resize(show_scores, show_scores, cv::Size(), reshape_factor,
             reshape_factor, cv::INTER_NEAREST);
  cv::applyColorMap(show_scores, show_scores, cv::COLORMAP_JET);

  cv::imshow("Similarity score", show_scores);

  // Only show matches which achieve score larger than score_filter.
  const float score_filter = 0.3;
  Eigen::MatrixXf scores_max(scores.rows(), scores.cols());
  scores_max.setZero();
  for (int i = 0; i < scores.cols(); ++i) {
    int max_index;
    if (scores.col(i).maxCoeff(&max_index) > score_filter)
      scores_max(max_index, i) = 1;
  }
  cv::Mat show_scores_max;
  cv::eigen2cv(scores_max, show_scores_max);
  cv::resize(show_scores_max, show_scores_max, cv::Size(), reshape_factor,
             reshape_factor, cv::INTER_NEAREST);

  cv::imshow("Max scores", show_scores_max);
  cv::waitKey();
}

}


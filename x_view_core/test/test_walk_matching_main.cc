#include <gtest/gtest.h>

#include "test_walk_matching.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

#include <Eigen/Core>
#include <glog/logging.h>

using namespace x_view;

TEST(XViewSlamTestSuite, test_walk_matching) {

  LOG(INFO) << "\n\n====Testing walk matching====";

  const int num_classes = 4;
  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_classes));

  const int num_vertices = 10;
  const float edge_probability = 0.4;
  const int num_walks = 100;
  const int walk_length = 3;

  Graph g;
  Graph::GraphType& graph = g.graph();
  graph = generateRandomGraph(num_vertices, edge_probability, num_classes);

  // Random walker parameters
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = walk_length;
  random_walker_params.num_walks_ = num_walks;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME
      ;

  RandomWalker random_walker(graph, random_walker_params);

  Eigen::MatrixXf scores(num_vertices, num_vertices);
  scores.setZero();

  const auto& mapped_walks = random_walker.getMappedWalks();

  // Set the similarity score type to be used.
  VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

  for (int i = 0; i < num_vertices; ++i) {
    const auto& vertex_d_i = boost::vertex(i, graph);
    const auto& vertex_i = graph[vertex_d_i];
    const auto& mapped_walks_i = mapped_walks[i];
    for (int j = i ; j < num_vertices; ++j) {
      const auto& vertex_d_j = boost::vertex(j, graph);
      const auto& vertex_j = graph[vertex_d_j];
      // Score is only nonzero if the source vertex has same semantic label.
      if (vertex_i.semantic_label_ == vertex_j.semantic_label_) {
        const auto& mapped_walks_j = mapped_walks[j];
        const float score =
            VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
        scores(i, j) = scores(j, i) = score;
      }
    }
  }
}


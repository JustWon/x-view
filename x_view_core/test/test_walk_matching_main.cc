#include <gtest/gtest.h>

#include "test_walk_matching.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_walk_matching) {

  LOG(INFO) << "\n\n====Testing walk matching====";

  const int num_semantic_classes = 3;
  LOG(INFO) << "Testing walk matching with " << num_semantic_classes
            << "classes.";

  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));
  CHECK_NOTNULL(global_dataset_ptr.get());

  const int num_vertices = 10;
  const float edge_probability = 0.2;
  const int num_walks = 1000;
  const int walk_length = 3;

  // Generate a random graph with the parameters specified above.
  Graph graph1 =
      generateRandomGraph(num_vertices, edge_probability, num_semantic_classes);

  // Copy the generated graph into a new graph instance.
  Graph graph2 = graph1;

  const int seed = 0;
  std::mt19937 rng(seed);

  // Define parameters for modifying the graph.
  GraphModifierParams params;
  params.num_vertices_to_add_ = 12;
  params.num_links_for_new_vertices_ = 3;
  params.num_vertices_to_remove_ = 5;
  params.num_edges_to_add_ = 0;
  params.num_edges_to_remove_ = 0;

  // Effectively add and remove vertices and edges from the graph.
  modifyGraph(&graph2, params, rng);

  std::cout << "Graph 1:\n" << graph1 << std::endl;
  std::cout << "Graph 2:\n" << graph2 << std::endl;

  std::cout << "Done"<<std::endl;

  // Random walker parameters
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = walk_length;
  random_walker_params.num_walks_ = num_walks;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  random_walker_params.force_visiting_each_neighbor_ = true;
  random_walker_params.allow_returning_back_ = false;

  RandomWalker random_walker1(graph1, random_walker_params);
  random_walker1.generateRandomWalks();

  RandomWalker random_walker2(graph2, random_walker_params);
  random_walker2.generateRandomWalks();

  // Retrieve the parameters passed to the RandomWalker as some of them
  // might have changed due to input coherence.
  random_walker_params = random_walker1.params();

  Eigen::MatrixXf scores(boost::num_vertices(graph1),
                         boost::num_vertices(graph2));
  scores.setZero();

  const auto& mapped_walks1 = random_walker1.getMappedWalks();
  const auto& mapped_walks2 = random_walker2.getMappedWalks();

  // Set the similarity score type to be used.
  VertexSimilarity::setScoreType(VertexSimilarity::SCORE_TYPE::HARD);

  for (int i = 0; i < boost::num_vertices(graph1); ++i) {
    const auto& vertex_d_i = boost::vertex(i, graph1);
    const auto& vertex_i = graph1[vertex_d_i];
    const auto& mapped_walks_i = mapped_walks1[i];
    for (int j = 0; j < boost::num_vertices(graph2); ++j) {
      const auto& vertex_d_j = boost::vertex(j, graph2);
      const auto& vertex_j = graph2[vertex_d_j];
      // Score is only nonzero if the source vertex has same semantic label.
      if (vertex_i.semantic_label_ == vertex_j.semantic_label_) {
        const auto& mapped_walks_j = mapped_walks2[j];
        const float score =
            VertexSimilarity::score(mapped_walks_i, mapped_walks_j);
        scores(i, j) = score;
      }
    }
  }

  std::cout << "Similarity matrix:\n" << scores << std::endl;
}


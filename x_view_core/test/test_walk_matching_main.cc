#include <gtest/gtest.h>

#include "test_walk_matching.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>
#include <x_view_core/matchers/graph_matcher/vertex_similarity.h>

#include <boost/graph/random.hpp>
#include <Eigen/Core>
#include <glog/logging.h>

#include <random>

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
  Graph g1;
  Graph::GraphType& graph1 = g1.graph();
  graph1 =
      generateRandomGraph(num_vertices, edge_probability, num_semantic_classes);

  // Copy the generated graph into a new graph instance.
  Graph g2;
  Graph::GraphType& graph2 = g2.graph();
  graph2 = graph1;

  const int seed = 0;
  std::mt19937 rng(seed);

  const auto& old_vertex = boost::random_vertex(graph2, rng);
  Graph::VertexProperty new_vertex;
  new_vertex.index_ = static_cast<int>(boost::num_vertices(graph2));
  new_vertex.semantic_label_ = 1;
  new_vertex.semantic_entity_name_ = "Newly entered vertex";
  const auto new_vertex_d = boost::add_vertex(new_vertex, graph2);
  boost::add_edge(new_vertex_d, old_vertex,
                  {new_vertex.index_, graph2[old_vertex].index_}, graph2);

  auto random_edge = boost::random_edge(graph2, rng);
  boost::remove_edge(random_edge, graph2);

  std::cout << "Vertices of graph 1:\n";
  g1.printVertices();
  std::cout << "Edges of graph 1:\n";
  g1.printEdges();

  std::cout << "Vertices of graph 2:\n";
  g2.printVertices();
  std::cout << "Edges of graph 2:\n";
  g2.printEdges();


  // Random walker parameters
  RandomWalkerParams random_walker_params;
  random_walker_params.walk_length_ = walk_length;
  random_walker_params.num_walks_ = num_walks;
  random_walker_params.random_sampling_type_ =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  random_walker_params.force_visiting_each_neighbor_ = true;
  random_walker_params.allow_returning_back_= false;

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


#include <gtest/gtest.h>

#include "test_random_walk.h"

#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/progress.hpp>
#include <x_view_core/landmarks/graph_landmark.h>

#include <chrono>

using namespace x_view;

TEST(XViewSlamTestSuite, test_random_walk) {

  const int num_semantic_classes = 3;
  LOG(INFO) << "Testing random walks with " << num_semantic_classes
            << "classes.";

  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));
  CHECK_NOTNULL(global_dataset_ptr.get());

  typedef Graph::VertexProperty Vertex;
  typedef Graph::VertexDescriptor VertexDescriptor;

  Graph g;
  Graph::GraphType& graph = g.graph();

  // create multiple random graphs with different topology and test them.
  std::vector<int> num_desired_vertices{10, 100, 300};
  std::vector<float> edge_probabilities{0.0, 0.01, 0.15};
  std::vector<RandomWalkerParams::RANDOM_SAMPLING_TYPE> sampling_types{
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM,
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME
  };

  for (const int num_vertices : num_desired_vertices) {
    for (const float edge_probability : edge_probabilities) {
      const int walk_length = 3;
      const int num_walks_per_vertex = 100;

      graph = generateRandomGraph(num_vertices,
                                  edge_probability,
                                  num_semantic_classes);

      LOG(INFO) << "Testing random graph with " << boost::num_vertices(graph)
                << " vertices, " << boost::num_edges(graph) << " edges.";

      for (const auto sampling_type : sampling_types) {

        RandomWalkerParams params;
        params.random_sampling_type_ = sampling_type;
        params.walk_length_ = walk_length;
        params.num_walks_ = num_walks_per_vertex;

        // Generate random walks for the graph and measure execution time.
        auto t1 = std::chrono::high_resolution_clock::now();
        RandomWalker random_walker(graph, params);
        auto t2 = std::chrono::high_resolution_clock::now();
        LOG(INFO) << "Generated " << num_walks_per_vertex
                  << " walks per vertex of length " << walk_length << " in "
                  << std::chrono::duration_cast<std::chrono::duration<double>>
                      (t2 - t1).count() << " seconds.";
        testTransitionProbabilityMatrix(random_walker, graph, params);
        testRandomWalkSequence(random_walker, graph, params);
      }
      LOG(INFO) << "Test passed.";
    }
  }
}


#include <gtest/gtest.h>

#include "test_random_walk.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>

#include <chrono>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_random_walk) {

  LOG(INFO) << "\n\n====Testing random walks generation====";

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

  const int walk_length = 3;
  const int num_walks_per_vertex = 20;

  // create multiple random graphs with different topology and test them.
  std::vector<std::pair<int, float>> graph_statistics{
      {10, 0.5},  // graph statistic has form (num_vertices, edge_probability)
      {10, 1.0},
      {50, 0.2},
      {50, 0.5},
      {300, 0.05}
  };
  std::vector<RandomWalkerParams::RANDOM_SAMPLING_TYPE> sampling_types{
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM,
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME
  };

  std::vector<bool> force_visiting_neighbors{true, false};

  int num_vertices;
  float edge_probability;
  for (auto graph_statistic : graph_statistics) {
    std::tie(num_vertices, edge_probability) = graph_statistic;

    graph = generateRandomGraph(num_vertices,
                                edge_probability,
                                num_semantic_classes);

    LOG(INFO) << "Testing random graph with " << boost::num_vertices(graph)
              << " vertices, " << boost::num_edges(graph) << " edges.";

    for (const auto sampling_type : sampling_types) {
      for (const bool force_visiting_neighbor : force_visiting_neighbors) {

        RandomWalkerParams params;
        params.random_sampling_type_ = sampling_type;
        params.walk_length_ = walk_length;
        params.num_walks_ = num_walks_per_vertex;
        params.force_visiting_each_neighbor_ = force_visiting_neighbor;

        // Generate random walks for the graph and measure execution time.
        RandomWalker random_walker(graph, params);
        auto t1 = std::chrono::high_resolution_clock::now();
        random_walker.generateRandomWalks();
        auto t2 = std::chrono::high_resolution_clock::now();
        LOG(INFO) << "Generated " << num_walks_per_vertex
                  << " walks for each of " << num_vertices << " vertices "
                  << " of length " << walk_length << " in "
                  << std::chrono::duration_cast<std::chrono::duration<double>>
                      (t2 - t1).count() << " seconds.";

        testTransitionProbabilityMatrix(random_walker, graph, params);
        testRandomWalkSequence(random_walker, graph, params);
        if (params.random_sampling_type_ ==
            RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME)
          testAvoidingStrategy(random_walker, graph, params);
        if(params.force_visiting_each_neighbor_)
          testVisitingNeighbors(random_walker, graph, params);

      }
    }
    LOG(INFO) << "Test passed.";
  }

}


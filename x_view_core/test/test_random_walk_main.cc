#include <gtest/gtest.h>

#include "test_random_walk.h"

#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/progress.hpp>

using namespace x_view;

TEST(XViewSlamTestSuite, test_random_walk) {

  const int num_semantic_classes = 3;
  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));

  typedef Graph::VertexProperty Vertex;
  typedef Graph::VertexDescriptor VertexDescriptor;

  Graph g;
  Graph::GraphType& graph = g.graph();
  std::vector<int> num_desired_vertices{8, 15, 50, 500};
  std::vector<float> edge_probabilities{0.1, 0.5, 0.8};
  boost::progress_display show_progress(num_desired_vertices.size() *
                                            edge_probabilities.size(),
                                        std::cout,
                                        "Transition probabilities\n");
  for (const int num_vertices : num_desired_vertices)
    for (const float edge_probability : edge_probabilities) {

      graph = generateRandomGraph(num_vertices,
                                  edge_probability,
                                  num_semantic_classes);

      testTransitionProbabilityMatrix(graph);
      ++show_progress;
    }
}


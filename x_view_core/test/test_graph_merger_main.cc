#include <gtest/gtest.h>

#include "test_graph_merger.h"
#include "test_common.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_tools.h>

#include <boost/graph/random.hpp>

#include <random>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_graph_merger) {

  LOG(INFO) << "\n\n======Testing graph merger======";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing graph landmark merger with " << num_semantic_classes
            << "classes.";

  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));
  CHECK_NOTNULL(global_dataset_ptr.get());

  const unsigned long seed = 0;

  // Generate a base graph.

  GraphConstructionParams graph_construction_params;
  graph_construction_params.num_semantic_classes =
      global_dataset_ptr->numSemanticClasses();
  graph_construction_params.seed = seed;
  graph_construction_params.num_vertices = 15;
  graph_construction_params.edge_probability = 0.2;

  Graph g1 = generateRandomGraph(graph_construction_params);

  // Extract a subgraph.
  std::mt19937 rng(seed);
  const int radius = 2;
  const VertexDescriptor random_v_d = boost::random_vertex(g1, rng);
  Graph g2_original = extractSubgraphAroundVertex(g1, random_v_d, radius);

  // Modify the extracted subgraph.
  GraphModifierParams graph_modifier_params;
  graph_modifier_params.num_vertices_to_add = 0;
  graph_modifier_params.num_links_for_new_vertices = 0;
  graph_modifier_params.num_vertices_to_remove = 0;
  graph_modifier_params.num_edges_to_add = 0;
  graph_modifier_params.num_edges_to_remove = 0;
  // Set the index at which the newly inserted vertices start.
  int max_index = std::numeric_limits<int>::min();
  const auto vertices = boost::vertices(g1);
  for(auto iter = vertices.first; iter != vertices.second; ++iter) {
    max_index = std::max(max_index, g1[*iter].index);
  }
  graph_modifier_params.start_vertex_index = max_index + 1;

  Graph g2 = g2_original;
  modifyGraph(&g2, graph_modifier_params, rng);

  // Perform the matching
  Graph merged_graph;
  mergeGraphs(g1, g2, &merged_graph);

  const std::string output_path = getOutputDirectory();
  const std::string g1_path = output_path + "g1.dot";
  const std::string g2_path = output_path + "g2.dot";
  const std::string merged_path = output_path + "merged.dot";

  dumpToDotFile(g1, g1_path);
  dumpToDotFile(g2, g2_path);
  dumpToDotFile(merged_graph, merged_path);
}


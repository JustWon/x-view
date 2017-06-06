#include <gtest/gtest.h>

#include "test_walk_matching.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_walk_matching) {

  LOG(INFO) << "\n\n====Testing walk matching====";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing walk matching with " << num_semantic_classes
            << "classes.";

  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));
  CHECK_NOTNULL(global_dataset_ptr.get());

  const unsigned long seed = 0;

  // Graph construction parameters
  const int num_vertices = 500;
  const float edge_probability = 0.005;

  // Graph modifier parameters
  const int num_vertices_to_add = 0;
  const int num_links_for_new_vertices = 0;
  const int num_vertices_to_remove = 0;
  const int num_edges_to_add = 0;
  const int num_edges_to_remove = 0;

  // Size of the extracted subgraph.
  const int extraction_radius = 2;

  // Random walker params
  const int num_walks = 1000;
  const int walk_length = 3;
  const RandomWalkerParams::RANDOM_SAMPLING_TYPE sampling_type =
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME;
  const bool force_visiting_each_neighbor = true;
  const bool allow_returning_back = true;

  // Define parameter for generating the graphs.
  GraphConstructionParams construction_params;
  construction_params.num_vertices_ = num_vertices;
  construction_params.edge_probability_ = edge_probability;
  construction_params.num_semantic_classes_ = num_semantic_classes;
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


  /////////// CHAIN GRAPH ///////////

  GraphPair graph_pair_chain = generateChainGraphPair(construction_params,
                                                      modifier_params,
                                                      extraction_radius);
  computeVertexSimilarity(graph_pair_chain, random_walker_params);


  /////////// RANDOM GRAPH ///////////

  GraphPair graph_pair_random = generateRandomGraphPair(construction_params,
                                                        modifier_params,
                                                        extraction_radius);
  computeVertexSimilarity(graph_pair_random, random_walker_params);

}


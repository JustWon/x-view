#ifndef X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H
#define X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H
#include "test_common.h"

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <Eigen/Core>

#include <random>

using namespace x_view;

namespace x_view_test {

/**
 * \brief Tests a chain-like graph.
 * \param seed Seed used to randomly generate the chain-like graph.
 */
void testChainGraph(const unsigned long seed);

/**
 * \brief Tests a graph with random topology.
 * \param seed Seed used to randomly generate the random graph.
 */
void testRandomGraph(const unsigned long seed);

/**
 * \brief Parameters used to modify the topology of a graph.
 */
struct GraphModifierParams {
  /// \brief Number of new vertices to add to the graph.
  int num_vertices_to_add;
  /// \brief Number of vertices to remove from the graph.
  int num_vertices_to_remove;
  /// \brief Number of new edges to add to the graph.
  int num_edges_to_add;
  /// \brief Number of edges to remove from the graph.
  int num_edges_to_remove;
  /// \brief Number of edges to create between each new vertex and the
  /// existing ones.
  int num_links_for_new_vertices = 2;
};

/**
 * \brief Small container used to combine a base graph with a new subgraph.
 */
struct GraphPair {
  Graph base_graph;
  Graph sub_graph;
};

/**
 * \brief Generates a new graph with a chain topology.
 * \param construction_params Parameters used to construct the graph.
 * \param modifier_params Parameters used to modify the subgraph extracted
 * from the generated base_graph.
 * \param extraction_radius Radius used for graph extraction.
 * \return A GraphPair object, containing the generated graph with
 * corresponding extracted subgraph.
 */
GraphPair generateChainGraphPair(const GraphConstructionParams& construction_params,
                                 const GraphModifierParams& modifier_params,
                                 const int extraction_radius);

/**
 * \brief Generates a new graph with a random topology.
 * \param construction_params Parameters used to construct the graph.
 * \param modifier_params Parameters used to modify the subgraph extracted
 * from the generated base_graph_.
 * \param extraction_radius Radius used for graph extraction.
 * \return A GraphPair object, containing the generated graph with
 * corresponding extracted subgraph.
 */
GraphPair generateRandomGraphPair(const GraphConstructionParams& construction_params,
                                  const GraphModifierParams& modifier_params,
                                  const int extraction_radius);

float similarityAccuracy(const GraphPair& graph_pair,
                         const Eigen::MatrixXf& similarity_matrix);

/**
 * \brief Modifies the graph pointed by the passed argument following the
 * parameters contained in the second argument.
 * \param graph Pointer pointing to the graph to be modified.
 * \param params Parameters used for modifying the graph.
 * \param rng Random number generator used to randomly create/remove
 * vertices/edges.
 */
void modifyGraph(Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng);

}

#endif //X_VIEW_TEST_GRAPH_LANDMARK_MATCHER_H

#ifndef X_VIEW_TEST_WALK_MATCHING_H
#define X_VIEW_TEST_WALK_MATCHING_H
#include "test_common.h"

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <Eigen/Core>

#include <random>

using namespace x_view;

namespace x_view_test {

/**
 * \brief Tests a chain-like grpah a
 * \param seed
 */
void testChainGraph(const unsigned long seed);

void testRandomGraph(const unsigned long seed);

/**
 * \brief Parameters used to modify the topology of a graph.
 */
struct GraphModifierParams {
  /// \brief Number of new vertices to add to the graph.
  int num_vertices_to_add_;
  /// \brief Number of vertices to remove from the graph.
  int num_vertices_to_remove_;
  /// \brief Number of new edges to add to the graph.
  int num_edges_to_add_;
  /// \brief Number of edges to remove from the graph.
  int num_edges_to_remove_;
  /// \brief Number of edges to create between each new vertex and the
  /// existing ones.
  int num_links_for_new_vertices_ = 2;
};

/**
 * \brief Small container used to combine a base graph with a new subgraph.
 */
struct GraphPair {
  Graph base_graph_;
  Graph sub_graph_;
};

/**
 * \brief Generates a new graph with a chain topology.
 * \param construction_params Parameters used to construct the graph.
 * \param modifier_params Parameters used to modify the subgraph extracted
 * from the generated base_graph_.
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

/**
 * \brief Routine computing vertex similarities between the graphs contained
 * in the passed argument.
 * \param graph_pair Object containing the two graph to be compared.
 * \param random_walker_params Parameters used to extract random walks from
 * the graphs passed as argument.
 * \return A similarity matrix whose entry (i,j) corresponds to the
 * similarity between node 'i' in graph_pair.base_graph_ and node 'j' in
 * graph_pair.sub_graph_.
 */
Eigen::MatrixXf computeVertexSimilarity(const GraphPair& graph_pair,
                                        const RandomWalkerParams& random_walker_params);

/**
 * \brief Displays the similarity matrix passed as argument as a heatmap
 * converting it into a cv::Mat object and computing a colormap on the entries.
 * \param similarity_matrix Similarity matrix to be displayed.
 * \param name Name to be used when displaying the similarity matrix.
 * \param desired_size Desired size of shown image.
 */
void displaySimilarityMatrix(const Eigen::MatrixXf& similarity_matrix,
                             const std::string& name = "",
                             const int desired_size = 400);

/**
 * \brief Displays the filtered similarity matrix such that each column of
 * the resulting heat map only shows the highest match, i.e. shows which
 * vertex represented in the vertical axis of the similarity matrix is the most
 * similar to each vertex in the horizontal axis of the similarity matrix.
 * \param similarity_matrix Similarity matrix to be displayed.
 * \param filter_val Only similarities larger than this parameter are shown
 * in the image.
 * \param name Name to be used when displaying the similarity matrix.
 * \param desired_size Desired size of shown image.
 */
void displayFilteredSimilarityMatrix(const Eigen::MatrixXf& similarity_matrix,
                                     const float filter_val = 0.0f,
                                     const std::string& name = "",
                                     const int desired_size = 400);

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

#endif //X_VIEW_TEST_WALK_MATCHING_H

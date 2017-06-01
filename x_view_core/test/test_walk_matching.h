#ifndef X_VIEW_TEST_WALK_MATCHING_H
#define X_VIEW_TEST_WALK_MATCHING_H

#include <x_view_core/features/graph.h>

#include <random>

using namespace x_view;

namespace x_view_test {

void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int link_to_n_vertices = 2);

void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng);

void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng);

void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng);

struct GraphModifierParams {
  int num_vertices_to_add_;
  int num_vertices_to_remove_;
  int num_edges_to_add_;
  int num_edges_to_remove_;

  int num_links_for_new_vertices_ = 2;
};

void modifyGraph(Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng);

}

#endif //X_VIEW_TEST_WALK_MATCHING_H

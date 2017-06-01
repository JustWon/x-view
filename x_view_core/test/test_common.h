#ifndef X_VIEW_TEST_COMMON_H
#define X_VIEW_TEST_COMMON_H

#include <x_view_core/features/graph.h>

namespace x_view_test {
/**
 * \brief Generates a random graph with num_vertices vertices, where each
 * pair of vertices is linked by an edge with probability edge_probability. A
 * random semantic class is associated to each vertex of the graph.
 * \param num_vertices Number of vertices of the generated graph.
 * \param edge_probability Probability to generate an edge between each pair
 * of vertices.
 * \param num_semantic_classes Number of semantic classes. Each vertex is
 * associated to a random semantic class (random integer in {0, ..,
 * num_semantic_classes-1}
 * \return The randomly generated graph.
 */
x_view::Graph generateRandomGraph(const int num_vertices,
                                  const float edge_probability,
                                  const int num_semantic_classes);
}

#endif //X_VIEW_TEST_COMMON_H

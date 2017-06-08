#ifndef X_VIEW_TEST_COMMON_H
#define X_VIEW_TEST_COMMON_H

#include <x_view_core/features/graph.h>

#include <boost/graph/breadth_first_search.hpp>

#include <map>
#include <random>

namespace x_view_test {

struct GraphConstructionParams {

  GraphConstructionParams()
      : num_vertices_(10),
        edge_probability_(0.1),
        num_semantic_classes_(13),
        seed_(0) {}

  /// \brief Number of vertices of the generated graph.
  int num_vertices_;
  /// \brief Probability to generate an edge between each pair of vertices.
  float edge_probability_;
  /// \brief Number of semantic classes. Each vertex is associated to a
  /// random semantic class (random integer in {0, .., num_semantic_classes-1}
  int num_semantic_classes_;
  /// \brief Seed to be used for graph generation.
  unsigned long seed_;
};

/**
 * \brief Generates a random graph with num_vertices vertices, where each
 * pair of vertices is linked by an edge with probability edge_probability. A
 * random semantic class is associated to each vertex of the graph.
 * \param params Parameters used for graph construction.
 */
x_view::Graph generateRandomGraph(const GraphConstructionParams& params);

/**
 * \brief Generates a chain-like graph with num_vertices each of
 * which is assigned to a random semantic class between 0 and
 * num_semantic_classes - 1.
 * \param params Parameters used for graph construction.
 */
x_view::Graph generateChainGraph(const GraphConstructionParams& params);

/**
 * \brief An instance of this class is used to build a subgraph based on K-hops.
 */
class KhopVisitor : public boost::default_bfs_visitor {

 public:
  /// \brief Map which assigns to each VertexDescriptor an integer distance
  /// (number of edges/hops) needed to reach the source vertex in a graph.
  typedef std::map<const x_view::VertexDescriptor, int> DistanceMap;

  KhopVisitor(DistanceMap& dist)
      : dist_(dist) {}

  /**
   * \brief Function called each time an edge is added to the search tree.
   * \details This function is used to compute the distance from the target
   * vertex of the added edge to the source vertex of the BFS algorithm.
   */
  template<typename EdgeT, typename GraphT>
  void tree_edge(EdgeT e, GraphT& g) {
    // Distance to the target it equal to distance to the source (of the
    // edge) increased by one.
    dist_[boost::target(e, g)] = dist_[boost::source(e, g)] + 1;
  }

 private:
  DistanceMap& dist_;
};

/**
 * \brief Extracts a subgraph from a larger graph, defined by a source vertex
 * and all vertices having a maximal distance from the source defined by the
 * passed parameters.
 * \param original Original graph from which the subgraph is extracted.
 * \param source Vertex descriptor indicating which vertex has to be used for
 * graph extraction.
 * \param radius Number of edges to traverse for graph extraction.
 * \return A new graph containing all vertices which lie at most at radius
 * edges of distance from the source vertex passed as argument.
 */
x_view::Graph extractSubgraphAroundVertex(const x_view::Graph& original,
                                          const x_view::VertexDescriptor& source,
                                          const int radius);

}

#endif //X_VIEW_TEST_COMMON_H

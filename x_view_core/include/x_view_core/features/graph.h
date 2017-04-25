#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace x_view {
/**
 * \brief namespace containing graph structure and functions related to graphs
 * \note for an explanation see http://blog.hostilefork.com/boost-graph-templates-not-crazy/
 */
class Graph {

 public:

/// \brief Property associated to a graph vertex
struct VertexProperty {
  VertexProperty(const int label) : semantic_label_(label) {}

  /// \brief semantic label associated to this graph vertex. This label
  /// corresponds to the same specified in the dataset description
  const int semantic_label_;
};

/// \brief Property associated to a graph edge
struct EdgeProperty {
  EdgeProperty() {}
};


/**
 * \brief A graph object represented as an adjacency list
 * \details first parameter: what stl container is used to store the edges
 * second parameter: what stl container is used to store the graph vertices
 * third parameter: directed or undirected graph type
 * fourth parameter: properties associated to each node
 * fifth parameter: properties associated to each edge
 */
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                              VertexProperty, EdgeProperty> Graph_;
  Graph(){}

  int numVertices() const {
    return int(boost::num_vertices(graph_));
  }

  /*
  const VertexProperty vertex(const int n) const {
    return boost::vertex(n, graph_);
  }

  void add_edge(const VertexProperty& i, const VertexProperty& j) {
    boost::add_edge(i, j, graph_);
  }
   */

 private:
  Graph_ graph_;

};

}

#endif //X_VIEW_GRAPH_H

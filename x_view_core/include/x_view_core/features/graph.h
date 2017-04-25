#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

namespace x_view {
/**
 * \brief namespace containing graph structure and functions related to graphs
 * \note for an explanation see http://blog.hostilefork.com/boost-graph-templates-not-crazy/
 */
class Graph {

 public:

  /// \brief Property associated to a graph vertex
  struct VertexProperty {
    /// \brief semantic label associated to this graph vertex. This label
    /// corresponds to the same specified in the dataset description
    int semantic_label_;
    /// \brief name of semantic entity associated to this vertex
    std::string semantic_entity_name_;

    const std::string vertexInfo() const {
      return std::string("(v) " + std::to_string(semantic_label_) + ": " +
          semantic_entity_name_);
    }
  };

  /// \brief Property associated to a graph edge
  struct EdgeProperty {
    // FIXME: this variable is unused, has to be removed
    int edge_index_;

    const std::string edgeInfo() const {
      return std::string("(e) " + std::to_string(edge_index_));
    }
  };

  /**
  * \brief A graph object represented as an adjacency list
  * \details first parameter: what stl container is used to store the edges
  * second parameter: what stl container is used to store the graph vertices
  * third parameter: directed or undirected graph type
  * fourth parameter: node representation
  * fifth parameter: edge representation
  */
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                                VertexProperty, EdgeProperty> GraphType;

  typedef boost::graph_traits<GraphType>::vertex_descriptor VertexDescriptor;
  typedef boost::graph_traits<GraphType>::edge_descriptor EdgeDescriptor;

  Graph() {}

  int numVertices() const {
    return int(boost::num_vertices(graph_));
  }

  void printVertices() const {
    auto iterators = boost::vertices(graph_);
    for (auto it = iterators.first; it != iterators.second; ++it) {
      auto vertex = graph_[*it];
      std::cout << vertex.vertexInfo() << std::endl;
    }
  }

  void printEdges() const {
    auto iterators = boost::edges(graph_);
    for (auto it = iterators.first; it != iterators.second; ++it) {
      auto edge = graph_[*it];
      std::cout << edge.edgeInfo() << std::endl;
    }
  }

  void print(std::ostream& out = std::cout) const {
    out << "\n-- graphviz output START --" << std::endl;
    boost::write_graphviz(out, graph_);
    out << "\n-- graphviz output END --" << std::endl;
  }

  const GraphType& graph() const {
    return graph_;
  }

  GraphType& graph() {
    return graph_;
  }

 private:
  GraphType graph_;

};

}

#endif //X_VIEW_GRAPH_H

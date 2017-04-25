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

  /// \brief Access to vertices stored in the graph
  typedef boost::graph_traits<GraphType>::vertex_descriptor VertexDescriptor;
  /// \brief Access to edges stored in the graph
  typedef boost::graph_traits<GraphType>::edge_descriptor EdgeDescriptor;

  /// \brief returns the number of vertices stored in the graph.
  int numVertices() const;

  /**
   * \brief prints the vertices (with vertex parameters) in a human readable
   * way to the stream passed as argument
   * \param out stream to be used to print the vertices
   */
  void printVertices(std::ostream& out = std::cout) const;

  /**
   * \brief prints the edges (with edge parameters) in a human readable way
   * to the stream passed as argument.
   * \param out stream to be used to print the vertices
   */
  void printEdges(std::ostream& out = std::cout) const;

  /**
   * \brief prints the graph structure/topology to the stream passed as argument
   * \param out stream to be used to print the vertices
   */
  void print(std::ostream& out = std::cout) const;

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

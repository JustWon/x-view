#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_traits.hpp>

#include <opencv2/core/core.hpp>

namespace x_view {
/**
 * \brief Class containing graph structure and functions related to graphs.
 * \note For an explanation see http://blog.hostilefork
 * .com/boost-graph-templates-not-crazy/.
 */
class Graph {

 public:

  /// \brief Property associated to a graph vertex.
  struct VertexProperty {
    /// \brief index of the vertex in the graph, such that
    /// boost::vertex(i, graph) returns the vertex property with index_ == i.
    int index_;
    /// \brief Semantic label associated to this graph vertex. This label
    /// corresponds to the same specified in the dataset description.
    int semantic_label_;
    /// \brief Name of semantic entity associated to this vertex.
    std::string semantic_entity_name_;
    /// \brief Number of pixels contained in this vertex/blob.
    int size_;
    /// \brief Blob center.
    cv::Point center_;

    const std::string vertexInfo() const {
      return std::string("(v) " + std::to_string(semantic_label_) + ": " +
          semantic_entity_name_ + ", size: " + std::to_string(size_) + ", "
          "center: [" + std::to_string(center_.x) + ", " + std::to_string
          (center_.y) + "]");
    }
  };

  /// \brief Property associated to a graph edge.
  struct EdgeProperty {
    int from_;
    int to_;

    const std::string edgeInfo() const {
      return std::string("(e) " + std::to_string(from_) + "->" +
          std::to_string(to_));
    }
  };

  /**
  * \brief A graph object represented as an adjacency list.
  * \details
  * First parameter: what stl container is used to store the edges. Using
   * setS as edge container to enforce uniqueness of edges, i.e. edge 1-2 is
   * the same as edge 2-1.
  * Second parameter: what stl container is used to store the graph vertices.
  * Third parameter: directed or undirected graph type.
  * Fourth parameter: node representation.
  * Fifth parameter: edge representation.
  */
  typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                                VertexProperty, EdgeProperty> GraphType;

  /// \brief Access to vertices stored in the graph.
  typedef boost::graph_traits<GraphType>::vertex_descriptor VertexDescriptor;
  /// \brief Access to edges stored in the graph.
  typedef boost::graph_traits<GraphType>::edge_descriptor EdgeDescriptor;

  /// \brief Returns the number of vertices stored in the graph.
  int numVertices() const;

  /**
   * \brief Prints the vertices (with vertex parameters) in a human readable
   * way to the stream passed as argument.
   * \param out Stream to be used to print the vertices.
   */
  void printVertices(std::ostream& out = std::cout) const;

  /**
   * \brief Prints the edges (with edge parameters) in a human readable way
   * to the stream passed as argument.
   * \param out Stream to be used to print the vertices
   */
  void printEdges(std::ostream& out = std::cout) const;

  /**
   * \brief Prints the graph structure/topology to the stream passed as argument
   * \param out Stream to be used to print the vertices
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

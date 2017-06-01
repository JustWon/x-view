#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <opencv2/core/core.hpp>

namespace x_view {

/// \brief Property associated to a graph vertex.
struct VertexProperty {
  /// \brief Index of the vertex in the graph, such that
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
};

/// \brief Property associated to a graph edge.
struct EdgeProperty {
  /// \brief Index of the first vertex defining this edge.
  int from_;
  /// \brief Index of the second vertex defining this edge.
  int to_;
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
 * \note The second parameter must be boost::vecS such that each vertex can
 * be accessed directly by the program as follows:
 * \code{.cpp}
 * Graph graph = createSomeGraph();
 * VertexProperty first_vertex = graph[0];
 * VertexProperty tenth_vertex = graph[9];
 * \endcode
 * Otherwise each time one wants to access a vertex it must perform a
 * linear/logarithmic search on the vertex list:
 * \code{.cpp}
 * Graph graph = createSomeGraph();
 * VertexProperty tenth_vertex;
 * auto vertex_iter = boost::vertices(graph);
 * for(int i = 0; vertex_iter.first != vertex_iter.second;
 *     ++i,++vertex_iter.first) {
 *     if(i == 9)
 *         tenth_vertex = graph[*vertex_iter.first];
 * }
 *\endcode
 */
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS,
                              VertexProperty, EdgeProperty> Graph;

/// \brief Access to vertices stored in the graph.
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

/// \brief Access to edges stored in the graph.
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

/**
 * \brief Tests if the i-th and the j-th vertex of the graph passed as
 * parameter are linked by an edge.
 * \param v1 Index of the first vertex.
 * \param v2 Index of the second vertex.
 * \param graph Graph containing the two vertices passed as argument.
 * \return True if an edge exists between vertex i and vertex j, false
 * otherwise.
 */
bool areVerticesConnected(const int v1, const int v2, const Graph& graph);

/// \brief Overloaded operator to print a vertex.
std::ostream& operator<<(std::ostream& out, const VertexProperty& v);

/// \brief Overloaded operator to print an edge.
std::ostream& operator<<(std::ostream& out, const EdgeProperty& e);

/// \brief Overloaded operator to print a graph.
std::ostream& operator<<(std::ostream& out, const Graph& graph);

}

#endif //X_VIEW_GRAPH_H

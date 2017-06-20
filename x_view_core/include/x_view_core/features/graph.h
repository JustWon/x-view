#ifndef X_VIEW_GRAPH_H
#define X_VIEW_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <boost/graph/breadth_first_search.hpp>
#include <opencv2/core/core.hpp>

#include <random>

namespace x_view {

/// \brief Property associated to a graph vertex.
struct VertexProperty {
  /// \brief Index of the vertex in the graph, such that
  /// boost::vertex(i, graph) returns the vertex property with index == i.
  int index;
  /// \brief Semantic label associated to this graph vertex. This label
  /// corresponds to the same specified in the dataset description.
  int semantic_label;
  /// \brief Name of semantic entity associated to this vertex.
  std::string semantic_entity_name;
  /// \brief Number of pixels contained in this vertex/blob.
  int num_pixels;
  /// \brief Blob center.
  cv::Point center;
};

/// \brief Property associated to a graph edge.
struct EdgeProperty {
  /// \brief Index of the first vertex defining this edge.
  int from;
  /// \brief Index of the second vertex defining this edge.
  int to;
};

/**
 * \brief A graph object represented as an adjacency list.
 * \details
 * First parameter: what stl container is used to store the edges. Using
 * setS as edge container to enforce uniqueness of edges, i.e. edge 1-2 is
 * the same as edge 2-1.
 * Second parameter: what stl container is used to store the graph vertices.
 * Third parameter: directed or undirected graph type.
 * Forth parameter: node representation.
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

/// \brief Access to vertices stored in the graph. Since vertices are stored
/// in a boost::vecS container, their VertexDescriptor corresponds to the
/// index at which they are stored: a VertexDescriptor is an unsigned long type.
typedef boost::graph_traits<Graph>::vertex_descriptor VertexDescriptor;

/// \brief Access to edges stored in the graph.
typedef boost::graph_traits<Graph>::edge_descriptor EdgeDescriptor;

/**
 * \brief Tests if the i-th and the j-th vertex of the graph passed as
 * parameter are linked by an edge.
 * \param v1 Index got by graph[v_1_d].index_ of first vertex descriptor.
 * \param v2 Index got by graph[v_2_d].index_ of second vertex descriptor.
 * \param graph Graph containing the two vertices passed as argument.
 * \return True if an edge exists between vertex with index_ = i
 * and vertex with index_ = j, false otherwise.
 * \note This approach is inefficient as it uses a member of the
 * VertexProperty to check the edge existence, consider using
 * VertexDescriptors whenever possible:
 * \code{.cpp}
 * Graph graph = getSomeGraph();
 * const VertexDescriptor v_1_d = getFirstVertexDescriptor();
 * const VertexDescriptor v_2_d = getSecondVertexDescriptor();
 * // Efficient way:
 * bool vertices_connected_1 = boost.:edge(v_1_d, v_2_d, graph).second;
 * // Inefficient way:
 * const VertexProperty v_1_p = graph[v_1 _d];
 * const VertexProperty v_2_p = graph[v_2 _d];
 * bool vertices_connected_2 =
 *    areVerticesConnectedByIndex(v_1_p.index_, v_2_p.index_, graph);
 *
 * assert(vertices_connected_1  == vertices_connected_2);
 * \endcode
 */
bool areVerticesConnectedByIndex(const int v1, const int v2,
                                 const Graph& graph);

/**
 * \brief Adds a new generated VertexProperty to the graph pointed by the
 * passed argument. The newly generated vertex is linked towards
 * link_to_n_vertices existing vertices of the graph.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 * \param link_to_n_vertices The added vertex is linked to link_to_n_vertices
 * randomly chosen vertices of the graph. This ensure that the new graph
 * consists of a single connected component.
 */
void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int link_to_n_vertices = 2);

/**
 * \brief Adds a new generated EdgeProperty to the graph pointed by the
 * passed argument. The edge is defined by randomly selecting two different
 * vertices of the graph passed as argument, and is added to the graph only
 * if the resulting edge does not already exist.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 */
void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Adds an edge between the VertexDescriptors passed as argument if
 * the edge does not exist yet.
 * \param v_1_d VertexDescriptor of first vertex.
 * \param v_2_d VertexDescriptor of second vertex.
 * \param graph Pointer to graph an edge is added to.
 * \return True if the edge has been added, false if the edge was already
 * present.
 */
bool addEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                            const VertexDescriptor& v_2_d, Graph* graph);

/**
 * \brief Removes a random vertex from the graph pointed by the passed argument.
 * This function makes sure that removing the vertex from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes a random edge from the graph pointed by the passed argument.
 * This function makes sure that removing the edge from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes the edge between the vertex descriptors passed as argument
 * and returns true only if the vertex has been removed correctly.
 * \param v_1_d First vertex descriptor.
 * \param v_2_d Second vertex descriptor.
 * \param graph Pointer to graph to be modified.
 * \return True if the function succeeded in removing the edge, false
 * otherwise.
 */
bool removeEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                               const VertexDescriptor& v_2_d, Graph* graph);


/// \brief Overloaded operator to print a vertex.
std::ostream& operator<<(std::ostream& out, const VertexProperty& v);

/// \brief Overloaded operator to print an edge.
std::ostream& operator<<(std::ostream& out, const EdgeProperty& e);

/// \brief Overloaded operator to print a graph.
std::ostream& operator<<(std::ostream& out, const Graph& graph);

}

#endif //X_VIEW_GRAPH_H

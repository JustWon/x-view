#include "test_walk_matching.h"

#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>

namespace x_view_test {

void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int link_to_n_vertices) {
  CHECK_NOTNULL(graph);

  // Create the new vertex.
  VertexProperty new_vertex;
  new_vertex.index_ = static_cast<int>(boost::num_vertices(*graph));
  // Random semantic label.
  new_vertex.semantic_label_ =
      static_cast<int>(rng() % global_dataset_ptr->numSemanticClasses());
  new_vertex.semantic_entity_name_ =
      "Random_edge" + std::to_string(new_vertex.index_);

  // Define random vertices to be linked with the new vertex.
  std::vector<VertexDescriptor> vertices_to_link;
  for (int i = 0; i < link_to_n_vertices; ++i)
    vertices_to_link.push_back(boost::random_vertex(*graph, rng));

  const VertexDescriptor& new_vertex_d = boost::add_vertex(new_vertex, *graph);
  std::cout << "Added vertex " << (*graph)[new_vertex_d].index_ << std::endl;
  for (const VertexDescriptor& v_d : vertices_to_link) {
    const VertexProperty& v_p = (*graph)[v_d];
    std::cout << "--> linked to existing vertex " << v_p.index_ << std::endl;
    boost::add_edge(v_d, new_vertex_d, {v_p.index_, new_vertex.index_}, *graph);
  }
}

void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  // Select two random vertices from the graph.
  const VertexDescriptor& v1_d = boost::random_vertex(*graph, rng);
  VertexDescriptor v2_d = boost::random_vertex(*graph, rng);
  while (v1_d == v2_d)
    v2_d = boost::random_vertex(*graph, rng);

  const VertexProperty& v1_p = (*graph)[v1_d];
  const VertexProperty& v2_p = (*graph)[v2_d];

  std::cout << "Added edge between vertices " << v1_p.index_ << ", " << v2_p
      .index_ << std::endl;
  boost::add_edge(v1_d, v2_d, {v1_p.index_, v2_p.index_}, *graph);
}

void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_vertices(*graph) <= 1) {
    LOG(WARNING) << "Cannot remove a vertex from a graph with "
                 << boost::num_vertices(*graph) << " vertices";
    return;
  }

  bool single_connected_component = false;
  while (!single_connected_component) {

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;

    // Select a random vertex from the graph.
    const VertexDescriptor v_d = boost::random_vertex(test_graph, rng);
    std::cout << "Trying to remove vertex " << test_graph[v_d].index_ <<
                                                                      std::endl;

    boost::clear_vertex(v_d, test_graph);
    boost::remove_vertex(v_d, test_graph);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      std::cout << "Succeeded"<<std::endl;
      single_connected_component = true;
      *graph = test_graph;
    }
  }
}

void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_edges(*graph) == 1)
    return;

  bool single_connected_component = false;
  while (!single_connected_component) {
    // Select a random vertex from the graph.
    const EdgeDescriptor e_d = boost::random_edge(*graph, rng);

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;
    boost::remove_edge(e_d, test_graph);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      single_connected_component = true;
      *graph = test_graph;
    }
  }
}

void modifyGraph(Graph* graph, const GraphModifierParams& params,
                 std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  std::vector<int> component(boost::num_vertices(*graph));
  int num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "Input graph to " << __FUNCTION__ << " has " << num_connected_components
  << " connected components. Function " << __FUNCTION__
  << " only works with single components.";

  for (int i = 0; i < params.num_vertices_to_add_; ++i)
    addRandomVertexToGraph(graph, rng, params.num_links_for_new_vertices_);

  for (int i = 0; i < params.num_vertices_to_remove_; ++i)
    removeRandomVertexFromGraph(graph, rng);

  for (int i = 0; i < params.num_edges_to_add_; ++i)
    addRandomEdgeToGraph(graph, rng);

  for (int i = 0; i < params.num_edges_to_add_; ++i) {
    removeRandomEdgeFromGraph(graph, rng);
  }

  component.resize(boost::num_vertices(*graph));
  num_connected_components =
      boost::connected_components(*graph, &component[0]);

  CHECK(num_connected_components == 1)
  << "After removing and adding vertices/edges to the graph, there are "
  << "disconnected  components.";

}

}


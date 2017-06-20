#include <x_view_core/features/graph.h>
#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/random.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

namespace x_view {

bool areVerticesConnectedByIndex(const int v1, const int v2,
                                 const Graph& graph) {
  const auto edges = boost::edges(graph);
  bool edge_exists = false;
  for (auto edge = edges.first; edge != edges.second; ++edge) {
    const EdgeProperty e_p = graph[*edge];
    if ((e_p.from_ == v1 && e_p.to_ == v2) ||
        (e_p.from_ == v2 && e_p.to_ == v1)) {
      // An edge exists linking node with index v1 and node with index v2
      edge_exists = true;
      return edge_exists;
    }
  }
  return edge_exists;
}

void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int index,  const int link_to_n_vertices) {
  CHECK_NOTNULL(graph);

  // Create the new vertex.
  VertexProperty new_vertex;
  if(index == -1)
    new_vertex.index_ = static_cast<int>(boost::num_vertices(*graph));
  else
    new_vertex.index_ = index;
  // Random semantic label.
  new_vertex.semantic_label_ =
      static_cast<int>(rng() % global_dataset_ptr->numSemanticClasses());
  new_vertex.semantic_entity_name_ =
      "Random vertex " + std::to_string(new_vertex.index_);

  // Define random vertices to be linked with the new vertex.
  std::vector<VertexDescriptor> vertices_to_link;
  while (vertices_to_link.size() < link_to_n_vertices) {
    const VertexDescriptor link_v_d = boost::random_vertex(*graph, rng);
    if (std::find(vertices_to_link.begin(), vertices_to_link.end(), link_v_d)
        == vertices_to_link.end())
      vertices_to_link.push_back(link_v_d);
  }

  const VertexDescriptor& new_vertex_d = boost::add_vertex(new_vertex, *graph);
  LOG(INFO) << "Added vertex " << (*graph)[new_vertex_d];
  for (const VertexDescriptor& v_d : vertices_to_link) {
    const VertexProperty& v_p = (*graph)[v_d];
    LOG(INFO) << "--> linked to existing vertex " << v_p.index_ << std::endl;
    boost::add_edge(v_d, new_vertex_d, {v_p.index_, new_vertex.index_}, *graph);
  }
}

void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  bool edge_added = false;
  while (!edge_added) {
    // Select two random vertices from the graph.
    const VertexDescriptor& v1_d = boost::random_vertex(*graph, rng);
    const VertexDescriptor& v2_d = boost::random_vertex(*graph, rng);

    if (v1_d != v2_d && addEdgeBetweenVertices(v1_d, v2_d, graph)) {
      const VertexProperty& v1_p = (*graph)[v1_d];
      const VertexProperty& v2_p = (*graph)[v2_d];

      LOG(INFO) << "Added edge between vertices " << v1_p.index_
                << ", " << v2_p.index_ << std::endl;
      edge_added = true;
    }
  }
}

bool addEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                            const VertexDescriptor& v_2_d, Graph* graph) {

  CHECK_NOTNULL(graph);

  if (boost::edge(v_1_d, v_2_d, *graph).second) {
    LOG(WARNING) << "Edge between " << (*graph)[v_1_d] << " and "
                 << (*graph)[v_2_d] << " already exists";
    return false;
  } else {
    const VertexProperty& v_1_p = (*graph)[v_1_d];
    const VertexProperty& v_2_p = (*graph)[v_2_d];
    boost::add_edge(v_1_d, v_2_d, {v_1_p.index_, v_2_p.index_}, *graph);
    return true;
  }
}

void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_vertices(*graph) <= 1) {
    LOG(WARNING) << "Cannot remove a vertex from a graph with "
                 << boost::num_vertices(*graph) << " vertices.";
    return;
  }

  bool single_connected_component = false;
  while (!single_connected_component) {

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;

    // Select a random vertex from the graph.
    const VertexDescriptor v_d = boost::random_vertex(test_graph, rng);
    // Create a copy of the vertex property to be deleted as the vertex
    // descriptor will be invalidated after the remove_vertex call.
    const VertexProperty v_p = test_graph[v_d];

    boost::clear_vertex(v_d, test_graph);
    boost::remove_vertex(v_d, test_graph);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      LOG(INFO) << "Removed vertex " << v_p << ".";
      single_connected_component = true;
      *graph = test_graph;
    } else {
      LOG(WARNING)
          << "Could not remove vertex " << v_d
          << " as it would create two disconnected components. "
          << "Choosing a new vertex.";
    }
  }
}

void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng) {
  CHECK_NOTNULL(graph);

  if (boost::num_edges(*graph) <= 1) {
    LOG(WARNING) << "Cannot remove an edge from a graph with "
                 << boost::num_edges(*graph) << " edges.";
    return;
  }

  bool single_connected_component = false;
  while (!single_connected_component) {

    // Try to remove the vertex on a test graph.
    Graph test_graph = *graph;

    // Select a random edge to be removed.
    const EdgeDescriptor& e_d = boost::random_edge(test_graph, rng);

    // Compute the connected components of the new graph.
    std::vector<int> component(boost::num_vertices(test_graph));
    int num_connected_components =
        boost::connected_components(test_graph, &component[0]);

    if (num_connected_components == 1) {
      const EdgeProperty& e_p = test_graph[e_d];
      LOG(INFO) << "Removed edge " << e_p << ".";
      single_connected_component = true;
      *graph = test_graph;
    } else {
      const EdgeProperty& e_p = test_graph[e_d];
      LOG(WARNING)
          << "Could not remove edge " << e_p
          << " as it would create two disconnected components. "
          << "Choosing a new edge.";

    }
  }
}

bool removeEdgeBetweenVertices(const VertexDescriptor& v_1_d,
                               const VertexDescriptor& v_2_d, Graph* graph) {
  CHECK_NOTNULL(graph);

  if (!boost::edge(v_1_d, v_2_d, *graph).second) {
    LOG(WARNING) << "Edge between " << (*graph)[v_1_d] << " and "
                 << (*graph)[v_2_d] << " does not exist, cannot remove it.";
    return false;
  } else {

    boost::remove_edge(v_1_d, v_2_d, *graph);
    return true;
  }

}

std::ostream& operator<<(std::ostream& out, const VertexProperty& v) {
  const static unsigned long max_label_length =
      global_dataset_ptr->largestLabelSize() + 2;
  out << "(v) " << v.index_
      << ", label: " << std::right << std::setw(2) << std::setfill(' ')
      << v.semantic_label_
      << ", name: " << std::right << std::setw(max_label_length) << v
          .semantic_entity_name_
      << ", size: " << v.size_ << ", center: " << v.center_;

  return out;
}

std::ostream& operator<<(std::ostream& out, const EdgeProperty& e) {
  out << std::setfill(' ');
  out << "(e) " << std::right << std::setw(2) << e.from_ << "--"
      << std::left << std::setw(2) << e.to_;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Graph& graph) {
  out << "Vertices:\n";
  auto vertex_iterator = boost::vertices(graph);
  for (; vertex_iterator.first != vertex_iterator.second;
         ++vertex_iterator.first) {
    out << graph[*vertex_iterator.first] << "\n";
  }
  out << "\nEdges:\n";
  auto edge_iterator = boost::edges(graph);
  for (; edge_iterator.first != edge_iterator.second; ++edge_iterator.first) {
    out << graph[*edge_iterator.first] << "\n";
  }
  return out;
}

void dumpToDotFile(Graph& graph, const std::string& filename) {

  // Open a stream
  std::ofstream out(filename.c_str());
  CHECK(out.is_open())  << "Impossible to open/create file " << filename << ". "
  << "Make sure the destination folder exists.";

  // Create a property map to be used during writing of the graph.
  boost::dynamic_properties dp;
  dp.property("label", boost::get(&VertexProperty::semantic_label_, graph));
  dp.property("node_id", boost::get(&VertexProperty::index_, graph));

  boost::write_graphviz_dp(out, graph, dp);

}

}


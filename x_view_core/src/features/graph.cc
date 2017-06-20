#include <x_view_core/features/graph.h>
#include <x_view_core/datasets/abstract_dataset.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>

namespace x_view {

bool areVerticesConnectedByIndex(const int v1, const int v2,
                                 const Graph& graph) {
  const auto edges = boost::edges(graph);
  bool edge_exists = false;
  for (auto edge = edges.first; edge != edges.second; ++edge) {
    const EdgeProperty e_p = graph[*edge];
    if ((e_p.from == v1 && e_p.to == v2) ||
        (e_p.from == v2 && e_p.to == v1)) {
      // An edge exists linking node with index v1 and node with index v2
      edge_exists = true;
      return edge_exists;
    }
  }
  return edge_exists;
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
    boost::add_edge(v_1_d, v_2_d, {v_1_p.index, v_2_p.index}, *graph);
    return true;
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
  out << "(v) " << v.index
      << ", label: " << std::right << std::setw(2) << std::setfill(' ')
      << v.semantic_label
      << ", name: " << std::right << std::setw(max_label_length) << v
          .semantic_entity_name
      << ", num pixels: " << v.num_pixels << ", center: " << v.center;

  return out;
}

std::ostream& operator<<(std::ostream& out, const EdgeProperty& e) {
  out << std::setfill(' ');
  out << "(e) " << std::right << std::setw(2) << e.from << "--"
      << std::left << std::setw(2) << e.to;
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

}


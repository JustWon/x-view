#include <x_view_core/features/graph.h>
#include <x_view_core/datasets/abstract_dataset.h>

namespace x_view {

bool areVerticesConnected(const int v1, const int v2, const Graph& graph) {
  const x_view::VertexDescriptor& vi = boost::vertex(v1, graph);
  const x_view::VertexDescriptor& vj = boost::vertex(v2, graph);
  return boost::edge(vi, vj, graph).second;
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

}


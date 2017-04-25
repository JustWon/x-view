#include <x_view_core/features/graph.h>

namespace x_view {

int Graph::numVertices() const {
  return int(boost::num_vertices(graph_));
}

void Graph::printVertices(std::ostream& out) const {
  auto iterators = boost::vertices(graph_);
  for (auto it = iterators.first; it != iterators.second; ++it) {
    auto vertex = graph_[*it];
    out << vertex.vertexInfo() << std::endl;
  }
}

void Graph::printEdges(std::ostream& out) const {
  auto iterators = boost::edges(graph_);
  for (auto it = iterators.first; it != iterators.second; ++it) {
    auto edge = graph_[*it];
    out << edge.edgeInfo() << std::endl;
  }
}

void Graph::print(std::ostream& out) const {
  out << "\n-- graphviz output START --" << std::endl;
  boost::write_graphviz(out, graph_);
  out << "\n-- graphviz output END --" << std::endl;
}

}


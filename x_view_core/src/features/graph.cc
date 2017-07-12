#include <x_view_core/features/graph.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>

#include <boost/graph/connected_components.hpp>
#include <boost/graph/random.hpp>
#include <glog/logging.h>

#include <fstream>
#include <iomanip>


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

  const auto& dataset = Locator::getDataset();

  const static unsigned long max_label_length =
      dataset->largestLabelSize() + 2;
  out << "(v) " << v.index
      << ", label: " << std::right << std::setw(2) << std::setfill(' ')
      << v.semantic_label
      << ", name: " << std::right << std::setw(max_label_length) << v
          .semantic_entity_name
      << ", num pixels: " << v.num_pixels << ", center: " << v.center
      << ", 3D location: " << Eigen::RowVector3d(v.location_3d);

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


void writeToFile(const Graph& graph, const std::string& filename) {
  if(filename.substr(filename.find_last_of(".") + 1) != "dot") {
    LOG(WARNING) << "Filename <" << filename << "> used to write graph to file "
                 << "has different extension than <.dot>.";
  }

  std::ofstream out(filename.c_str());
  if(!out.is_open()) {
    LOG(ERROR) << "Impossible to open file <" << filename << ", graph is not "
        "written to file.";
    return;
  }

  auto scalarToString = [](const int s) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0') << std::setw(2) << s;
    return ss.str();
  };

  out << "graph semantic_graph {\n";

  // Iterate over the vertices of the graph.
  const auto vertices = boost::vertices(graph);
  for(auto iter = vertices.first; iter != vertices.second; ++iter) {
    const VertexProperty& v_p = graph[*iter];
    const cv::Scalar color =
        x_view::getColorFromSemanticLabel(v_p.semantic_label);
    out << "\t" << v_p.index << " ["
        " label=\"" << v_p.semantic_entity_name << "\"," <<
        " fillcolor=\"#" << scalarToString(color[0])
                    << scalarToString(color[1])
                    << scalarToString(color[2]) << "\","
        " fontcolor=\"#" << scalarToString(255-color[0])
                         << scalarToString(255-color[1])
                         << scalarToString(255-color[2]) << "\",";
    if(v_p.location_3d != Eigen::Vector3d::Zero()) {
      out << " pos = \"" << v_p.location_3d[0] << ", "
                  << v_p.location_3d[1] << "!\",";
    }
       out << " style=filled ]\n";
  }

  // Iterate over the edges of the graph.
  const auto edges = boost::edges(graph);
  for(auto iter = edges.first; iter != edges.second; ++iter) {
    const VertexProperty& from = graph[boost::source(*iter, graph)];
    const VertexProperty& to = graph[boost::target(*iter, graph)];

    out << "\t" << from.index << "--" << to.index << "\n";
  }

  out << "}";
}

}


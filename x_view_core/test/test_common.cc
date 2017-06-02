#include "test_common.h"

#include <boost/graph/random.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/random/linear_congruential.hpp>

namespace x_view_test {

x_view::Graph generateRandomGraph(const int num_vertices,
                                  const float edge_probability,
                                  const int num_semantic_classes) {

  typedef boost::erdos_renyi_iterator<boost::minstd_rand,
                                      x_view::Graph> ERGen;

  boost::minstd_rand gen;
  // Create random graph
  x_view::Graph graph(ERGen(gen, num_vertices, edge_probability),
                      ERGen(), num_vertices);

  // add properties to the vertices
  auto vertex_iter = boost::vertices(graph);
  int vertex_index = 0;
  // iterate over all vertices
  for (; vertex_iter.first != vertex_iter.second; ++vertex_iter.first) {
    auto& vertex = graph[*vertex_iter.first];
    vertex.index_ = vertex_index++;
    vertex.size_ = 1;
    vertex.center_ = cv::Point(0, 0);
    // set a random semantic label to the vertex.
    vertex.semantic_label_ = rand() & num_semantic_classes;
    vertex.semantic_entity_name_ = std::to_string(vertex.semantic_label_);
  }

  // add edge properties
  auto edges_iter = boost::edges(graph);
  for (; edges_iter.first != edges_iter.second; ++edges_iter.first) {
    // get the vertex descriptors defining the current edge
    const auto& from_v = graph[boost::source(*edges_iter.first, graph)];
    const auto& to_v = graph[boost::target(*edges_iter.first, graph)];
    // set the edge properties
    graph[*edges_iter.first].from_ = from_v.index_;
    graph[*edges_iter.first].to_ = to_v.index_;
  }

  return graph;
}
}


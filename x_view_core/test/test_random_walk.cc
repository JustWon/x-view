#include "test_random_walk.h"

#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <glog/logging.h>

#include <boost/graph/random.hpp>
#include <boost/graph/erdos_renyi_generator.hpp>
#include <boost/random/linear_congruential.hpp>

x_view::Graph::GraphType generateRandomGraph(const int num_vertices,
                                             const float edge_probability,
                                             const int num_semantic_classes) {

  typedef boost::erdos_renyi_iterator<boost::minstd_rand,
                                      x_view::Graph::GraphType> ERGen;

  boost::minstd_rand gen;
  // Create random graph
  x_view::Graph::GraphType graph(ERGen(gen, num_vertices, edge_probability),
                                 ERGen(), num_vertices);

  // add properties to the vertices
  auto vertex_iter = boost::vertices(graph);
  int vertex_index = 0;
  // iterate over all vertices
  for (vertex_iter.first; vertex_iter.first != vertex_iter.second;
       ++vertex_iter.first) {
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
  for (edges_iter.first; edges_iter.first != edges_iter.second;
       ++edges_iter.first) {
    // get the vertex descriptors defining the current edge
    const auto& from_v = graph[boost::source(*edges_iter.first, graph)];
    const auto& to_v = graph[boost::target(*edges_iter.first, graph)];
    // set the edge properties
    graph[*edges_iter.first].from_ = from_v.index_;
    graph[*edges_iter.first].to_ = to_v.index_;
  }

  return graph;
}

void testTransitionProbabilityMatrix(const x_view::Graph::GraphType graph,
                                     const x_view::RandomWalkerParams& params) {

  // Build the transition probability of the graph.
  x_view::RandomWalker random_walker(graph, params);
  const Eigen::SparseMatrix<float>& trans =
      random_walker.getTransitionProbabilityMatrix();

  const auto& random_walks = random_walker.getRandomWalks();

  // iterate over all vertices contained in the graph.
  auto vertex_iter = boost::vertices(graph);
  for (vertex_iter.first; vertex_iter.first != vertex_iter.second;
       ++vertex_iter.first) {
    // each vertex corresponds to a row in the transition probability matrix.
    // The row is determined by the index associated to the vertex.
    const x_view::Graph::VertexProperty& vi = graph[*vertex_iter.first];
    const int vertex_index = vi.index_;
    // count the non-zero elements in the corresponding row of the transition
    // probability matrix.
    int num_elements_in_row = 0;
    for (int j = 0; j < trans.cols(); ++j) {
      if (trans.coeff(vertex_index, j) > 0.f) {
        num_elements_in_row++;
      }
    }
    // Each non-zero element in the row should have value
    // 1.0/num_elements_in_row.
    const float should_have_value = 1.f / num_elements_in_row;
    for (int j = 0; j < trans.cols(); ++j) {
      const float v = trans.coeff(vertex_index, j);
      CHECK(v == 0.f || v == should_have_value)
          << "Probability matrix at (" << vertex_index << ", " << j
          << ") has value " << v << " but should either be 0 or "
          << should_have_value;
      // if transition probability is nonzero, then there must be an edge
      // between the corresponding vertices.
      if (v > 0)
        CHECK(boost::edge(*vertex_iter.first, boost::vertex(j, graph),
                          graph).second)
            << "Transition probability between vertex " << vertex_index
            << " and vertex " << j
            << " is nonzero but there is no edge between them.";
    }
  }

}


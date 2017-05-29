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

void testTransitionProbabilityMatrix(const x_view::RandomWalker& random_walker,
                                     const x_view::Graph::GraphType& graph,
                                     const x_view::RandomWalkerParams& params) {

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
      << should_have_value << ".";
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

void testRandomWalkSequence(const x_view::RandomWalker& random_walker,
                            const x_view::Graph::GraphType& graph,
                            const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      CHECK(areVerticesConnected(start_vertex_index,
                                 random_walk[0]->index_, graph) ||
          start_vertex_index == random_walk[0]->index_)
      << "Start vertex " << start_vertex_index << " and vertex "
      << random_walk[0]->index_ << " appear in a random walk "
      << "but there is no edge between them.";
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index_;
        const int to_index = random_walk[i + 1]->index_;
        CHECK(areVerticesConnected(from_index, to_index, graph) ||
            from_index == to_index)
        << "Vertex " << random_walk[i]->index_ << " and vertex "
        << random_walk[i + 1]->index_ << " appear in a random walk "
        << "but there is no edge between them.";
      }
    }
    ++start_vertex_index;
  }
}

bool areVerticesConnected(const int i, const int j,
                          const x_view::Graph::GraphType& graph) {
  const x_view::Graph::VertexDescriptor& vi = boost::vertex(i, graph);
  const x_view::Graph::VertexDescriptor& vj = boost::vertex(j, graph);
  return boost::edge(vi, vj, graph).second;
}


#include "test_random_walk.h"

#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <glog/logging.h>

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

void testAvoidingStrategy(const x_view::RandomWalker& random_walker,
                          const x_view::Graph::GraphType& graph,
                          const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index_;
        const int from_label = random_walk[i]->semantic_label_;
        const auto& from_vertex_descriptor = boost::vertex(from_index, graph);
        // Get the neighbors of the current vertex.
        auto from_vertex_neighbors =
            boost::adjacent_vertices(from_vertex_descriptor, graph);
        bool all_neighbors_have_same_label = true;
        // Check whether all neighbors have the same semantic label as the
        // current vertex or not.
        for (from_vertex_neighbors.first;
             from_vertex_neighbors.first != from_vertex_neighbors.second;
             ++from_vertex_neighbors.first) {
          if (graph[*from_vertex_neighbors.first].semantic_label_ !=
              from_label) {
            all_neighbors_have_same_label = false;
            break;
          }
        }
        // Only verify avoiding property if there is at least one neighbor
        // with different semantic label.
        if (!all_neighbors_have_same_label) {
          const int to_index = random_walk[i + 1]->index_;
          const int to_label = random_walk[i + 1]->semantic_label_;
          CHECK(from_label != to_label)
          << "Even though the RandomWalker class is using the avoiding "
          << "strategy, there is a random walk starting from vertex "
          << start_vertex_index << " with an edge between nodes "
          << from_index << " and " << to_index
          << " that have the same semantic label " << from_label;
        }
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


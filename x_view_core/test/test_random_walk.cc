#include "test_random_walk.h"

namespace x_view_test {

void testRandomWalkSequence(const x_view::RandomWalker& random_walker,
                            const x_view::Graph& graph,
                            const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      CHECK(areVerticesConnectedByIndex(start_vertex_index,
                                        random_walk[0]->index_, graph) ||
          start_vertex_index == random_walk[0]->index_)
      << "Start vertex " << start_vertex_index << " and vertex "
      << random_walk[0]->index_ << " appear in a random walk "
      << "but there is no edge between them.";
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index_;
        const int to_index = random_walk[i + 1]->index_;
        CHECK(areVerticesConnectedByIndex(from_index, to_index, graph) ||
            from_index == to_index)
        << "Vertex " << from_index << " and vertex " << to_index
        << " appear in a random walk at position " << i << " and " << i + 1
        << " respectively in the random walk starting at vertex "
        << start_vertex_index << " but there is no edge between them in the graph.";
      }
    }
    ++start_vertex_index;
  }
}

void testAvoidingStrategy(const x_view::RandomWalker& random_walker,
                          const x_view::Graph& graph,
                          const x_view::RandomWalkerParams& params) {
  const auto& all_random_walks = random_walker.getRandomWalks();
  int start_vertex_index = 0;
  for (const auto& random_walks : all_random_walks) {
    for (const auto& random_walk : random_walks) {
      for (int i = 0; i < random_walk.size() - 1; ++i) {
        const int from_index = random_walk[i]->index_;
        const int from_label = random_walk[i]->semantic_label_;
        // Using from_index as key for extracting vertex descriptor from
        // graph because in this test the index_ member of the VertexProperty
        // corresponds to the VertexDescriptor index associated to the
        // VertexProperty.
        const auto& from_vertex_descriptor = boost::vertex(from_index, graph);
        // Get the neighbors of the current vertex.
        auto from_vertex_neighbors =
            boost::adjacent_vertices(from_vertex_descriptor, graph);
        bool all_neighbors_have_same_label = true;
        // Check whether all neighbors have the same semantic label as the
        // current vertex or not.
        for (; from_vertex_neighbors.first != from_vertex_neighbors.second;
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

}

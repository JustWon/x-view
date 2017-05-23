#include <x_view_core/landmarks/graph_landmark/random_walker.h>

namespace x_view {

RandomWalker::RandomWalker(const Graph::GraphType& graph,
                           const RandomWalkerParams& params)
    : graph_(graph),
      params_(params) {
  precomputeTransitionProbabilities();
}

void RandomWalker::generateRandomWalks(const int num_walks,
                                       const int walk_length) {

}

void RandomWalker::precomputeTransitionProbabilities() {
  const unsigned long num_vertices = boost::num_vertices(graph_);
  // reset the transition probability matrix
  transition_probabilities_.resize(num_vertices, num_vertices);
  transition_probabilities_.data().squeeze();

  // iterate over all vertices of the graph and compute the transition
  // probability from that vertex to its neighbors.
  std::vector<Eigen::Triplet<float> > triplet_list;
  auto vertex_iter = boost::vertices(graph_);
  for (vertex_iter.first; vertex_iter.first != vertex_iter.second;
       ++vertex_iter.first) {
    // extract the current vertex from the graph
    const Graph::VertexProperty& vertex_i = graph_[*vertex_iter.first];
    const int index_i = vertex_i.index_;
    // compute the adjacent vertices of the current vertex v_index
    auto neighbors = boost::adjacent_vertices(*vertex_iter.first, graph_);
    auto num_neighbors = boost::degree(*vertex_iter.first, graph_);
    const float prob = 1.f / num_neighbors;
    std::vector<Eigen::Triplet<float> > row_triplets(num_neighbors);
    // iterate over the neighbors and add the implicitly defined vertex to
    // the transition probability matrix.
    for (int n_idx = 0; neighbors.first != neighbors.second;
         ++neighbors.first, ++n_idx) {
      const Graph::VertexProperty& vertex_j = graph_[*neighbors.first];
      const int index_j = vertex_j.index_;
      // add an edge between index_i and index_j
      row_triplets.push_back(Eigen::Triplet<float>(index_i, index_j, prob));
    }

    // add the row triplets to the global triplet list
    triplet_list.insert(triplet_list.end(), row_triplets.begin(),
                        row_triplets.end());
  }

  // build the sparse matrix from the triplet list.
  transition_probabilities_.setFromTriplets(triplet_list.begin(),
                                            triplet_list.end());

  // normalize each row of the transition probability matrix.

}

}


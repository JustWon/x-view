#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <glog/logging.h>

namespace x_view {

RandomWalker::RandomWalker(const Graph::GraphType& graph,
                           const RandomWalkerParams& params)
    : graph_(graph),
      params_(params) {
  switch (params_.random_sampling_type_) {
    case RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM: {
      precomputeUniformTransitionProbabilities();
      break;
    }
    case RandomWalkerParams::RANDOM_SAMPLING_TYPE::AVOID_SAME: {
      precomputeAvoidingTransitionProbabilities();
      break;
    }
    default: {
      CHECK(false) << "Unrecognized random sampling type passed to "
                   << __FUNCTION__;
    }
  }
}

void RandomWalker::generateRandomWalks(const int num_walks,
                                       const int walk_length) {

}

void RandomWalker::precomputeUniformTransitionProbabilities() {
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
    const unsigned long num_neighbors =
        boost::degree(*vertex_iter.first, graph_);
    if (num_neighbors > 0) {
      const float prob = 1.f / num_neighbors;
      // iterate over the neighbors and add the implicitly defined edge to
      // the transition probability matrix.
      for (int n_idx = 0; neighbors.first != neighbors.second;
           ++neighbors.first, ++n_idx) {
        const Graph::VertexProperty& vertex_j = graph_[*neighbors.first];
        const int index_j = vertex_j.index_;
        // add an edge between index_i and index_j
        triplet_list.push_back(Eigen::Triplet<float>(index_i, index_j, prob));
      }
    }
  }

  // build the sparse matrix from the triplet list.
  transition_probabilities_.setFromTriplets(triplet_list.begin(),
                                            triplet_list.end());
}

void RandomWalker::precomputeAvoidingTransitionProbabilities() {
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
    const int semantic_label_i = vertex_i.semantic_label_;
    // compute the adjacent vertices of the current vertex v_index
    auto neighbors = boost::adjacent_vertices(*vertex_iter.first, graph_);
    // count how many neighbors exist with different semantic label
    unsigned long num_neighbors_with_different_semantic_label = 0;
    std::vector<int> neighbor_indices_with_different_semantic_label;
    for (neighbors.first; neighbors.first != neighbors.second;
         ++neighbors.first) {
      const Graph::VertexProperty& neighbor = graph_[*neighbors.first];
      // only add neighbor if the semantic label is different
      if (neighbor.semantic_label_ != semantic_label_i) {
        num_neighbors_with_different_semantic_label++;
        const int neighbor_index = neighbor.index_;
        neighbor_indices_with_different_semantic_label.push_back(neighbor_index);
      }
    }
    if (num_neighbors_with_different_semantic_label > 0) {
      const float prob = 1.f / num_neighbors_with_different_semantic_label;
      // add the transition probability to the neighbors with different
      // semantic label
      for (const int neighbor_index :
          neighbor_indices_with_different_semantic_label) {
        // add an edge between index_i and index_j
        triplet_list.push_back(Eigen::Triplet<float>(index_i, neighbor_index,
                                                     prob));
      }
    }
  }

  // build the sparse matrix from the triplet list.
  transition_probabilities_.setFromTriplets(triplet_list.begin(),
                                            triplet_list.end());
}

}


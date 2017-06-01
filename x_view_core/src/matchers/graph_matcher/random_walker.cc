#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <glog/logging.h>
#include <x_view_core/datasets/abstract_dataset.h>

namespace x_view {

RandomWalker::RandomWalker(const Graph::GraphType& graph,
                           const RandomWalkerParams& params,
                           const int random_seed)
    : random_distribution_(0, 1),
      random_engine_(random_seed),
      graph_(graph),
      params_(params) {

  if (params_.random_sampling_type_ !=
      RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM &&
      params_.allow_returning_back_ == false) {
    LOG(WARNING) << "Random walker cannot use sampling type different than "
                 << "'UNIFORM' and at the same time restrict the walk to not "
                 << "returning back. Setting RandomWalkerParameter to:\n"
                 << "\t-Random sampling type: UNIFORM\n"
                 << "\t-Allow returning back: true";
    RandomWalkerParams& params_temp = const_cast<RandomWalkerParams&>(params_);
    params_temp.random_sampling_type_ =
        RandomWalkerParams::RANDOM_SAMPLING_TYPE::UNIFORM;
    params_temp.allow_returning_back_ = true;
  }

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

void RandomWalker::generateRandomWalks() {
  const unsigned long num_vertices = boost::num_vertices(graph_);
  random_walks_.clear();
  random_walks_.resize(num_vertices);

  mapped_walks_.clear();
  mapped_walks_.resize(num_vertices);

  auto vertex_iter = boost::vertices(graph_);
  for (vertex_iter.first; vertex_iter.first != vertex_iter.second;
       ++vertex_iter.first) {
    const Graph::VertexDescriptor start_vertex_descriptor = *vertex_iter.first;
    const Graph::VertexProperty& vertex_i = graph_[start_vertex_descriptor];
    const int vertex_index = vertex_i.index_;

    // Create a walk map object which stores the generated random walks keyed
    // by a unique identifier.
    WalkMap walk_map;
    walk_map.reserve(params_.num_walks_);

    const auto neighbors =
        boost::adjacent_vertices(start_vertex_descriptor, graph_);
    auto neighbor_iterator = neighbors.first;

    for (int w = 0; w < params_.num_walks_; ++w) {
      RandomWalk random_walk(params_.walk_length_);
      Graph::VertexDescriptor current_vertex_descriptor;
      int old_vertex_index = vertex_index;
      // Add the first vertex either by visiting all 1-ring neighbors, or by
      // sampling one randomly following the transition probability matrix.
      if (params_.force_visiting_each_neighbor_) {
        // Add the current neighbor as the first vertex of the random walk.
        const Graph::VertexProperty& current_neighbor_vertex_p =
            graph_[*neighbor_iterator];
        random_walk[0] = &current_neighbor_vertex_p;

        current_vertex_descriptor = *neighbor_iterator;

        // increase the neighbor_iterator for next random walk
        ++neighbor_iterator;
        if (neighbor_iterator == neighbors.second)
          neighbor_iterator = neighbors.first;

      } else { // The first step can be taken randomly.
        current_vertex_descriptor = nextVertex(vertex_index);
        const Graph::VertexProperty
            & vertex_j = graph_[current_vertex_descriptor];
        random_walk[0] = &vertex_j;
      }
      // Perform remaining steps starting from 1 because the first step has
      // already been taken.
      for (int step = 1; step < params_.walk_length_; ++step) {
        // Query the index of the current vertex and randomly sample the next
        // vertex to follow on the random_walk.
        const int
            current_vertex_index = graph_[current_vertex_descriptor].index_;
        Graph::VertexDescriptor next = nextVertex(current_vertex_index);
        int next_vertex_index = graph_[next].index_;

        unsigned long current_number_of_neighbors =
            boost::degree(current_vertex_descriptor, graph_);

        // If random walk cannot return to previously visited vertex and the
        // number of neighbors of current vertex is greater than one, then
        // check returning property.
        if (!params_.allow_returning_back_ && current_number_of_neighbors > 1) {
          // If next_vertex_index is the same as the one before current,
          // we need to find a new next vertex.
          while (next_vertex_index == old_vertex_index) {
            next = nextVertex(current_vertex_index);
            next_vertex_index = graph_[next].index_;
          }
        }

        // Add the vertex to the random_walk.
        const Graph::VertexProperty& vertex_j = graph_[next];
        random_walk[step] = &vertex_j;

        // Set the old vertex index accordingly.
        old_vertex_index = graph_[current_vertex_descriptor].index_;

        // Set the current graph vertex descriptor as the next so that it
        // can be used in the next iteration.
        current_vertex_descriptor = next;
      }

      // Add the generated random_walk to the random_walks_ container.
      random_walks_[vertex_index].push_back(random_walk);
    }

    // Iterate over the generated random walks of the current vertex_index and
    // map them to a unique identifier.
    for (const RandomWalk& random_walk : random_walks_[vertex_index]) {
      const int walk_id = RandomWalker::computeRandomWalkKey(random_walk);
      // Check whether this id has already been added to the walk_map
      WalkMap::iterator found_position = walk_map.find(walk_id);
      if (found_position == walk_map.end()) // New id.
        walk_map.insert({walk_id, MappedWalk(random_walk)});
      else // Increase multiplicity of already inserted MappedWalk with same id.
        ++(found_position->second);

      // Set the walk_map to the global mapped_walks_ container
      mapped_walks_[vertex_index] = walk_map;
    }

  }
}

const Graph::VertexDescriptor RandomWalker::nextVertex(
    const int current_vertex_index) const {
  const float probability = random_distribution_(random_engine_);

  // iterate over the elements of the current_vertex_index-th row of the
  // transition probability matrix and select the vertex sampled with the
  // computed probability
  int sampled_vertex_index = -1;
  float cumulative_probability = 0.f;
  for (int j = 0; j < transition_probabilities_.cols(); ++j) {
    const float
        val = transition_probabilities_.coeff(current_vertex_index, j);
    if (val > 0.f) {
      cumulative_probability += val;
      if (cumulative_probability >= probability) {
        sampled_vertex_index = j;
        break;
      }
    }
  }
  // if there this vertex has no connections to any other edge
  if (sampled_vertex_index == -1)
    return boost::vertex(current_vertex_index, graph_);

  const Graph::VertexDescriptor next_vertex_descriptor =
      boost::vertex(sampled_vertex_index, graph_);

#ifdef X_VIEW_DEBUG
  // sampled_vertex_index is the index of the next vertex in the path
  const Graph::VertexDescriptor current_vertex_descriptor =
      boost::vertex(current_vertex_index, graph_);

  // test if an edge exists between the two vertices
  CHECK(boost::edge(current_vertex_descriptor, next_vertex_descriptor, graph_)
            .second) << "Vertex " << current_vertex_index << " and "
                     << sampled_vertex_index << " are not neighbors";
#endif
  return next_vertex_descriptor;
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
      for (neighbors.first; neighbors.first != neighbors.second;
           ++neighbors.first) {
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
        neighbor_indices_with_different_semantic_label.push_back(
            neighbor_index);
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
    }// if all neighbor nodes have the same label, sample randomly between them
    else {
      // reset the neighbor iterators
      neighbors = boost::adjacent_vertices(*vertex_iter.first, graph_);
      const float prob = 1.f / boost::degree(*vertex_iter.first, graph_);
      for (neighbors.first; neighbors.first != neighbors.second;
           ++neighbors.first) {
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

const int RandomWalker::computeRandomWalkKey(const RandomWalk& random_walk) {
  const static int num_classes = global_dataset_ptr->numSemanticClasses();
  int id = 0;
  int mult = 1;
  for (const auto& val : random_walk) {
    id += mult * val->semantic_label_;
    mult *= num_classes;
  }
  return id;
}

}


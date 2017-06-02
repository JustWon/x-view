#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <x_view_core/datasets/abstract_dataset.h>

namespace x_view {

#ifdef X_VIEW_DEBUG
#define DEFAULT_NUM_RANDOM_WALKS 10
#define DEFAULT_RANDOM_WALK_LENGTH 2
#else
#define DEFAULT_NUM_RANDOM_WALKS 100
#define DEFAULT_RANDOM_WALK_LENGTH 3
#endif

RandomWalkerParams::RandomWalkerParams()
    : random_sampling_type_(RANDOM_SAMPLING_TYPE::UNIFORM),
      num_walks_(DEFAULT_NUM_RANDOM_WALKS),
      walk_length_(DEFAULT_RANDOM_WALK_LENGTH),
      force_visiting_each_neighbor_(false),
      allow_returning_back_(true) {
}

#undef DEFAULT_NUM_RANDOM_WALKS
#undef DEFAULT_RANDOM_WALK_LENGTH

std::ostream& operator<<(std::ostream& out, const RandomWalkerParams& params) {

  out << "Number of random walks:  " << params.num_walks_ << std::endl;
  out << "Walk length:             " << params.walk_length_ << std::endl;
  out << "Force neighbor visiting: "
      << (params.force_visiting_each_neighbor_ ? "true" : "false")
      << std::endl;
  out << "Allow returning back:    "
      << (params.allow_returning_back_ ? "true" : "false") << std::endl;
  out << "Random sampling type:    "
      << static_cast<int>(params.random_sampling_type_) << std::endl;
  return out;

}

RandomWalker::RandomWalker(const Graph& graph,
                           const RandomWalkerParams& params,
                           const int random_seed)
    : random_distribution_(0, 1),
      random_engine_(random_seed),
      graph_(graph),
      params_(params) {

  // Check that the graph type being used has a vertex_list_selector which
  // corresponds to boost::vecS! If an other selector would be used, the
  // implementation contained in this file does not work.
  static_assert(std::is_same<boost::vecS, Graph::vertex_list_selector>::value,
                "Random walker only works with Graphs "
                    "having with 'boost::vecS' vertex list selector "
                    "(second template argument).");

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

  // A random walk consists of a sequence of graph vertices. The variable
  // names used in this functions are associated to this figure:
  //
  //      A : source_vertex_index, the vertex where the random walk starts
  //      |
  //      v
  //      B : old_vertex_index: vertex visited previously than the
  //      |   current_vertex_index.
  //      v
  //      C : current_vertex_index : last vertex of the random walk.
  //      |
  //      v
  //      D : next_vertex_index : vertex which will be added to the random
  //          walk as a next step.

  for (VertexDescriptor source_vertex_index = 0;
       source_vertex_index < num_vertices; ++source_vertex_index) {

    // Create a walk map object which stores the generated random walks keyed
    // by a unique identifier.
    WalkMap walk_map;
    walk_map.reserve(params_.num_walks_);

    // Query the neighbors of the source vertex.
    const auto source_neighbors =
        boost::adjacent_vertices(source_vertex_index, graph_);
    auto source_neighbor_iterator = source_neighbors.first;

    // Create params_.num_walks random walks.
    for (int w = 0; w < params_.num_walks_; ++w) {
      RandomWalk random_walk(params_.walk_length_);

      // Prepare variables for iteration over the random walk.
      VertexDescriptor current_vertex_index;
      unsigned long old_vertex_index = source_vertex_index;

      // Add the first vertex either by visiting all 1-ring neighbors, or by
      // sampling one randomly following the transition probability matrix.

      // Visit a neighbor in the first step.
      if (params_.force_visiting_each_neighbor_) {
        // Add the current neighbor as the first vertex of the random walk.
        const VertexProperty& current_neighbor_vertex_p =
            graph_[*source_neighbor_iterator];
        random_walk[0] = &current_neighbor_vertex_p;

        // Set the current vertex index to the added neighbor.
        current_vertex_index = *source_neighbor_iterator;

        // Increase the source_neighbor_iterator for next random walk.
        ++source_neighbor_iterator;

        // Make sure to loop over all neighbors as a cycle.
        if (source_neighbor_iterator == source_neighbors.second)
          source_neighbor_iterator = source_neighbors.first;

      }
        // The first step can be taken randomly.
      else {
        // Randomly sample a vertex for the first step.
        current_vertex_index = nextVertex(source_vertex_index);
        const VertexProperty& current_vertex_p = graph_[current_vertex_index];
        random_walk[0] = &current_vertex_p;
      }
      // Perform remaining steps starting from 1 because the first step has
      // already been taken.
      for (int step = 1; step < params_.walk_length_; ++step) {

        VertexDescriptor next_vertex_index = nextVertex(current_vertex_index);

        unsigned long current_number_of_neighbors =
            boost::degree(current_vertex_index, graph_);

        // If random walk cannot return to previously visited vertex and the
        // number of neighbors of current vertex is greater than one, then
        // check returning property.
        if (!params_.allow_returning_back_ && current_number_of_neighbors > 1) {
          // If next_vertex_index is the same as the one before current,
          // we need to find a new next vertex.
          while (next_vertex_index == old_vertex_index) {
            next_vertex_index = nextVertex(current_vertex_index);
          }
        }

        // Add the vertex to the random_walk.
        const VertexProperty& next_vertex_p = graph_[next_vertex_index];
        random_walk[step] = &next_vertex_p;

        // Set the old vertex index accordingly.
        old_vertex_index = current_vertex_index;

        // Set the current graph vertex descriptor as the next so that it
        // can be used in the next iteration.
        current_vertex_index = next_vertex_index;
      }

      // Add the generated random_walk to the random_walks_ container.
      random_walks_[source_vertex_index].push_back(random_walk);
    }

    // Iterate over the generated random walks of the current vertex_index and
    // map them to a unique identifier.
    for (const RandomWalk& random_walk : random_walks_[source_vertex_index]) {
      const int walk_id = RandomWalker::computeRandomWalkKey(random_walk);
      // Check whether this id has already been added to the walk_map.
      WalkMap::iterator found_position = walk_map.find(walk_id);
      if (found_position == walk_map.end()) // New id.
        walk_map.insert({walk_id, MappedWalk(random_walk)});
      else // Increase multiplicity of already inserted MappedWalk with same id.
        ++(found_position->second);

      // Set the walk_map to the global mapped_walks_ container
      mapped_walks_[source_vertex_index] = walk_map;
    }
  }
}

const VertexDescriptor RandomWalker::nextVertex(
    const VertexDescriptor current_vertex_index) const {
  const float probability = random_distribution_(random_engine_);

  // Iterate over the elements of the current_vertex_index-th row of the
  // transition probability matrix and select the vertex sampled with the
  // computed probability.
  int sampled_vertex_index = -1;
  float cumulative_probability = 0.f;
  for (int j = 0; j < transition_probabilities_.cols(); ++j) {
    const float val = transition_probabilities_.coeff(current_vertex_index, j);
    if (val > 0.f) {
      cumulative_probability += val;
      if (cumulative_probability >= probability) {
        sampled_vertex_index = j;
        break;
      }
    }
  }
  // If this vertex has no connections to any other vertex.
  if (sampled_vertex_index == -1)
    return boost::vertex(current_vertex_index, graph_);

  const VertexDescriptor next_vertex_descriptor =
      boost::vertex(sampled_vertex_index, graph_);

  return next_vertex_descriptor;
}

void RandomWalker::precomputeUniformTransitionProbabilities() {
  const unsigned long num_vertices = boost::num_vertices(graph_);
  // Reset the transition probability matrix.
  transition_probabilities_.resize(num_vertices, num_vertices);
  transition_probabilities_.data().squeeze();

  // Iterate over all vertices of the graph and compute the transition
  // probability from that vertex to its neighbors.
  std::vector<Eigen::Triplet<float>> triplet_list;
  for (VertexDescriptor vertex_index_i = 0;
       vertex_index_i < num_vertices; ++vertex_index_i) {
    // Compute the adjacent vertices of the current vertex.
    const auto neighbors = boost::adjacent_vertices(vertex_index_i, graph_);
    const unsigned long num_neighbors = boost::degree(vertex_index_i, graph_);
    if (num_neighbors > 0) {
      const float prob = 1.f / num_neighbors;
      // Iterate over the neighbors and add the implicitly defined edge to
      // the transition probability matrix.
      for (auto neighbor_iterator = neighbors.first;
           neighbor_iterator != neighbors.second; ++neighbor_iterator) {
        // Add an edge between index_i and index_j
        triplet_list.push_back({vertex_index_i, *neighbor_iterator, prob});
      }
    }
  }

  // Build the sparse matrix from the triplet list.
  transition_probabilities_.setFromTriplets(triplet_list.begin(),
                                            triplet_list.end());
}

void RandomWalker::precomputeAvoidingTransitionProbabilities() {
  const unsigned long num_vertices = boost::num_vertices(graph_);
  // Reset the transition probability matrix.
  transition_probabilities_.resize(num_vertices, num_vertices);
  transition_probabilities_.data().squeeze();

  // Iterate over all vertices of the graph and compute the transition
  // probability from that vertex to its neighbors.
  std::vector<Eigen::Triplet<float>> triplet_list;
  for (VertexDescriptor vertex_index_i = 0;
       vertex_index_i < num_vertices; ++vertex_index_i) {
    // Extract the current vertex from the graph.
    const VertexProperty& vertex_i = graph_[vertex_index_i];
    const int semantic_label_i = vertex_i.semantic_label_;
    // Compute the adjacent vertices of the current vertex.
    const auto neighbors = boost::adjacent_vertices(vertex_index_i, graph_);
    const unsigned long num_neighbors = boost::degree(vertex_index_i, graph_);
    // Count how many neighbors exist with different semantic label.
    unsigned long num_neighbors_with_different_semantic_label = 0;
    std::vector<VertexDescriptor> neighbor_with_different_semantic_label;
    for (auto neighbor_iterator = neighbors.first;
         neighbor_iterator != neighbors.second; ++neighbor_iterator) {
      const VertexProperty& neighbor = graph_[*neighbor_iterator];
      // Only add neighbor if the semantic label is different.
      if (neighbor.semantic_label_ != semantic_label_i) {
        ++num_neighbors_with_different_semantic_label;
        // Add the neighbor to the neighbors with different semantic label
        neighbor_with_different_semantic_label.push_back(*neighbor_iterator);
      }
    }
    if (num_neighbors_with_different_semantic_label > 0) {
      const float prob = 1.f / num_neighbors_with_different_semantic_label;
      // Add the transition probability to the neighbors with different
      // semantic label.
      for (const VertexDescriptor& neighbor_index :
          neighbor_with_different_semantic_label) {
        // Add an edge between vertex_index_i and neighbor_index.
        triplet_list.push_back({vertex_index_i, neighbor_index, prob});
      }
    }// If all neighbor nodes have the same label, sample randomly between them.
    else {
      const float prob = 1.f / num_neighbors;
      for (auto neighbor_iterator = neighbors.first;
           neighbor_iterator != neighbors.second; ++neighbor_iterator) {
        // Add an edge between vertex_index_i and neighbor_iterator.
        triplet_list.push_back({vertex_index_i, *neighbor_iterator, prob});
      }
    }
  }

  // Build the sparse matrix from the triplet list.
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


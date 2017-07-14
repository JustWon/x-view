#include <x_view_core/matchers/graph_matcher/random_walker.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/x_view_locator.h>

namespace x_view {

#ifdef X_VIEW_DEBUG
#define DEFAULT_NUM_RANDOM_WALKS 100
#define DEFAULT_RANDOM_WALK_LENGTH 3
#else
#define DEFAULT_NUM_RANDOM_WALKS 500
#define DEFAULT_RANDOM_WALK_LENGTH 3
#endif

RandomWalkerParams::RandomWalkerParams()
    : random_sampling_type(SAMPLING_TYPE::UNIFORM),
      num_walks(DEFAULT_NUM_RANDOM_WALKS),
      walk_length(DEFAULT_RANDOM_WALK_LENGTH) {
}

#undef DEFAULT_NUM_RANDOM_WALKS
#undef DEFAULT_RANDOM_WALK_LENGTH

RandomWalkerParams::RandomWalkerParams(const SAMPLING_TYPE sampling_type,
                                       const int num_walks,
                                       const int walk_length)
    : random_sampling_type(sampling_type),
      num_walks(num_walks),
      walk_length(walk_length) {
}

std::ostream& operator<<(std::ostream& out, const RandomWalkerParams& params) {

  out << "Number of random walks:  " << params.num_walks << std::endl;
  out << "Walk length:             " << params.walk_length << std::endl;
  out << "Random sampling type:    "
      << static_cast<int>(params.random_sampling_type) << std::endl;
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

  LOG(INFO) << "Random walker is using parameters:\n" << params_;
}

void RandomWalker::generateRandomWalks() {
  const uint64_t num_vertices = boost::num_vertices(graph_);
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
    walk_map.reserve(params_.num_walks);

    // Create params_.num_walks random walks.
    for (int w = 0; w < params_.num_walks; ++w) {
      RandomWalk random_walk(params_.walk_length);

      // Prepare variables for iteration over the random walk.
      VertexDescriptor current_vertex_index = source_vertex_index;;

      for (int step = 0; step < params_.walk_length; ++step) {

        VertexDescriptor next_vertex_index = nextVertex(current_vertex_index);

        // Add the vertex to the random_walk.
        const VertexProperty& next_vertex_p = graph_[next_vertex_index];
        random_walk[step] = &next_vertex_p;

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
  const float p = random_distribution_(random_engine_);

  const uint64_t num_neighbor_vertices =
      boost::degree(current_vertex_index, graph_);
  if (num_neighbor_vertices == 0) {
    LOG(WARNING) << "Vertex " << current_vertex_index << " has no neighbors. "
        "Creating constant random walk.";
    return current_vertex_index;
  }

  auto neighbors = boost::adjacent_vertices(current_vertex_index, graph_);

  bool found_neighbor_with_different_label = true;
  if (params_.random_sampling_type ==
      RandomWalkerParams::SAMPLING_TYPE::AVOIDING) {
    const int current_label = graph_[current_vertex_index].semantic_label;
    std::vector<VertexDescriptor> different_index_v_d;
    different_index_v_d.reserve(num_neighbor_vertices);
    for (auto iter = neighbors.first; iter != neighbors.second; ++iter) {
      if (graph_[*iter].semantic_label != current_label)
        different_index_v_d.push_back(*iter);
    }
    const uint64_t num_neighbors_with_different_label =
        different_index_v_d.size();
    if (num_neighbors_with_different_label > 0) {
      const int advance_step =
          static_cast<int>(p * num_neighbors_with_different_label);
      return different_index_v_d[advance_step];
    } else {
      // All neighbors have same labels, so we are forced to sample neighbors
      // randomly.
      found_neighbor_with_different_label = false;
    }
  }
  if (params_.random_sampling_type ==
      RandomWalkerParams::SAMPLING_TYPE::UNIFORM ||
      !found_neighbor_with_different_label) {
    const int advance_step = static_cast<int>(p * num_neighbor_vertices);
    std::advance(neighbors.first, advance_step);
    return *neighbors.first;
  }
}

const int RandomWalker::computeRandomWalkKey(const RandomWalk& random_walk) {

  const auto& dataset = Locator::getDataset();

  const static int num_classes = dataset->numSemanticClasses();
  int id = 0;
  int mult = 1;
  for (const auto& val : random_walk) {
    id += mult * val->semantic_label;
    mult *= num_classes;
  }
  return id;
}

}


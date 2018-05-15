#ifndef X_VIEW_RANDOM_WALKER_H
#define X_VIEW_RANDOM_WALKER_H

#include <x_view_core_os/features/graph.h>
#include <x_view_core_os/x_view_types.h>

#include <Eigen/Sparse>
#include <glog/logging.h>

#include <random>
#include <unordered_map>
#include <vector>

namespace x_view {

/**
 * \brief Parameters used by the RandomWalk class.
 */
struct RandomWalkerParams {
  /// \brief Random walks can be either performed in a complete random way
  /// (i.e. by randomly sampling neighbors around each node belonging to the
  /// random walk) or using other techniques.
  enum class SAMPLING_TYPE {
        /// \brief True random walk.
        UNIFORM = 0,
        /// \brief Avoid transitions between vertices with same semantic label.
        AVOIDING,
        /// \brief Traverses edges with a probability proportional to the
        /// `num_times_seen` property.
        WEIGHTED,
        /// \brief This strategy tries to avoid 'returning' walks, i.e. random
        /// walks of the form 'A - B - A'.
        NON_RETURNING,
        /// \brief This strategy tries to increase distance to the source vertex.
        INCREASING_DISTANCE
  };

  /**
   * \brief Default random walk parameters.
   */
  RandomWalkerParams();

  RandomWalkerParams(const SAMPLING_TYPE sampling_type,
                     const int  num_walks, const int walk_length);

  /// \brief Determines the type of random walk to be generated.
  SAMPLING_TYPE random_sampling_type;
  /// \brief Number or random walks to generate for each node in the graph.
  int num_walks;
  /// \brief Random walk length (i.e. number of steps taken for each walk
  /// starting from the source node).
  int walk_length;
};

/**
 * \brief Streams the parameters in a human readable way to the passed
 * stream argument.
 * \param out Stream object to be streamed to.
 * \param params RandomWalkerParameter object to be streamed.
 * \return Stream filled with RandomWalkerParameters.
 */
std::ostream& operator<<(std::ostream& out, const RandomWalkerParams& params);

/**
 * \brief Class responsible for generating random walks on a graph.
 */
class RandomWalker {

 public:

  /// \brief A random walk is defined by the sequence of graph nodes defining
  /// the walk path.
  typedef std::vector<const VertexProperty*> RandomWalk;
  /// \brief Container for multiple random walks.
  typedef std::vector<RandomWalk> RandomWalks;

  /// \brief Aside from storing the random walks as a sequence of
  /// VertexProperty pointers, the RandomWalker also maps each walk to a unique
  /// integer identifier, that is used to check if two vertices share the
  /// same random walk. The value/object mapped by the key is the following
  /// struct capturing the mapped random walk, and the number of times the
  /// random walk appears for the source vertex.
  struct MappedWalk {
    const RandomWalk random_walk;
    int multiplicity;

    MappedWalk(const RandomWalk& random_walk)
        : random_walk(random_walk),
          multiplicity(1) {
    }

    /// \brief Utility operator to increase the multiplicity of by one.
    /// \details This operator is used whenever a new random walk is inserted to
    /// the walk map and the same walk ID is already present.
    MappedWalk& operator++() { // ++MappedWalk
      ++this->multiplicity;
      return *this;
    }
  };

  /// \brief A WalkMap maps a unique integer identifier to a random walk. The
  /// unique integer identifier is computed by 'hashing' the semantic labels
  /// of the random walk.
  typedef std::unordered_map<int, MappedWalk> WalkMap;

  /**
   * \brief Constructor of random walk object.
   * \param graph The random walks computed by this object are generated by
   * randomly iterating over the graph passed as argument.
   * \param params Parameters to be used by the RandomWalker class to
   * generate random walks.
   * \param random_seed Integer to be used as seed for random number generator.
   */
  RandomWalker(const Graph& graph,
               const RandomWalkerParams& params = RandomWalkerParams(),
               const int random_seed = 0);

  /**
   * \brief Generated num_walks random walks of length walk_length for each
   * node contained in the graph graph_. The length and number of random
   * walks generated for each graph vertex is determined by the parameters
   * passed to the constructor.
   */
  void generateRandomWalks();

  /**
   * \brief Access to the generated random walks.
   * \return A const reference to the random walks generated for the vertices
   * of the graph.
   */
  const std::vector<RandomWalks>& getRandomWalks() const {
    CHECK(random_walks_.size() > 0)
    << "Random walks might not have been computed. You might want to "
    << "call 'random_walker.generateRandomWalks()' first.";
    return random_walks_;
  }

  const std::vector<WalkMap>& getMappedWalks() const {
    CHECK(mapped_walks_.size() > 0)
    << "Random walks might not have been computed. You might want to "
    << "call 'random_walker.generateRandomWalks()' first.";
    return mapped_walks_;
  }

  /**
   * \brief Access to the random walks of a desired vertex.
   * \param vertex_index Index of the vertex source of the random walks.
   * \return A const reference to the random walks generated for the vertex
   * passed as argument.
   */
  const RandomWalks& getRandomWalksOfVertex(const int vertex_index) const {
    CHECK(vertex_index >= 0 && vertex_index < boost::num_vertices(graph_))
    << "Vertex index (" << vertex_index << ") passed to " << __FUNCTION__
    << "is either negative or larger than the number of vertices contained in"
        " the graph";
    return getRandomWalks()[vertex_index];
  }

  /**
   * \brief Access to the parameters being used by the RandomWalker.
   * \return Const reference to the parameters used by the RandomWalker.
   */
  const RandomWalkerParams& params() const { return params_; }

  /**
   * \brief Access to the graph over which the random walks are defined.
   * \return Const reference to the graph used by this RandomWalker instance.
   */
  const Graph& graph() const {return graph_;}

 private:
  /// \brief Distribution to be used when picking a new random number.
  /// Needs to be mutable as it is used inside a const member function.
  mutable std::uniform_real_distribution<real_t> random_distribution_;
  /// \brief Random engine used to generate new random numbers.
  /// Needs to be mutable as it is used inside a const member function.
  mutable std::mt19937 random_engine_;

  /**
   * \brief Computes a unique key associated to the random walk passed as
   * parameter. This key is used as key in the WalkMap structure.
   * \param random_walk Random walk whose key is to be computed.
   * \return Integer value used as key in the WalkMap structure.
   */
  static const int computeRandomWalkKey(const RandomWalk& random_walk);


 protected:

  /**
   * \brief Samples a new vertex from the set of neighbors of the current
   * vertex based on the transition probability matrix.
   * \param current_vertex_index Current vertex index of the random walk.
   * \param previous_vertex_index Previous vertex index of the random walk.
   * This vertex index is used in the computation of the next vertex if the
   * random walk strategy is set to 'NON_RETURNING'.
   * \return A new vertex being neighbor of current, sampled randomly
   * following the transition probabilities contained in
   * transition_probabilities_
   */
  const VertexDescriptor nextVertex(const VertexDescriptor current_vertex_index,
                                    const VertexDescriptor previous_vertex_index,
                                    const Vector3r& center_location);

  /// \brief All random walks generated by this RandomWalker instance are
  /// based on the graph graph_.
  const Graph& graph_;

  /// \brief Parameters used to generate the random walks.
  const RandomWalkerParams params_;

  /// \brief Container filled up with the random walks. The i-th element of
  /// random_walks_ is the set of random walks starting from the i-th vertex
  /// of the graph graph_.
  std::vector<RandomWalks> random_walks_;

  std::vector<WalkMap> mapped_walks_;

  // The currently travelled distance from the center vertex.
  real_t distance_from_center_;

};

}

#endif //X_VIEW_RANDOM_WALKER_H

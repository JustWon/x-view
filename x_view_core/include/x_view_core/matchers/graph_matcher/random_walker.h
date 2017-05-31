#ifndef X_VIEW_RANDOM_WALKER_H
#define X_VIEW_RANDOM_WALKER_H


// FIXME: this header and the corresponding source file must be moved into the matcher folder as they work together with the GraphMatcher class.

#include <x_view_core/features/graph.h>

#include <Eigen/Sparse>
#include <glog/logging.h>

#include <random>
#include <unordered_map>
#include <vector>

#ifdef X_VIEW_DEBUG
#define DEFAULT_NUM_RANDOM_WALKS 10
#define DEFAULT_RANDOM_WALK_LENGTH 2
#else
#define DEFAULT_NUM_RANDOM_WALKS 100
#define DEFAULT_RANDOM_WALK_LENGTH 3
#endif

namespace x_view {

/**
 * \brief Parameters used by the RandomWalk class.
 */
struct RandomWalkerParams {
  /// \brief Random walks can be either performed in a complete random way
  /// (i.e. by randomly sampling neighbors around each node belonging to the
  /// random walk) or using other techniques.
  enum class RANDOM_SAMPLING_TYPE {
    /// \brief True random walk.
        UNIFORM = 0,
    /// \brief Avoid transitions between vertices with same semantic label.
        AVOID_SAME
  };

  /**
   * \brief Default random walk parameters.
   */
  RandomWalkerParams()
      : random_sampling_type_(RANDOM_SAMPLING_TYPE::UNIFORM),
        num_walks_(DEFAULT_NUM_RANDOM_WALKS),
        walk_length_(DEFAULT_RANDOM_WALK_LENGTH),
        force_visiting_each_neighbor_(false) {
  }

  /// \brief Determines the type of random walk to be generated.
  RANDOM_SAMPLING_TYPE random_sampling_type_;
  /// \brief Number or random walks to generate for each node in the graph.
  int num_walks_;
  /// \brief Random walk length (i.e. number of steps taken for each walk
  /// starting from the source node).
  int walk_length_;
  /// \brief If set to true, then the first generated random walk step is
  /// forced to visit each of the neighbors of the source vertex. Otherwise the
  /// random walks are effectively random allowing the first step to always
  /// visit the same neighbor without ever visiting the others.
  bool force_visiting_each_neighbor_;
};

/**
 * \brief Class responsible for generating random walks on a graph.
 */
class RandomWalker {

 public:

  /// \brief A random walk is defined by the sequence of graph nodes defining
  /// the walk path.
  typedef std::vector<const Graph::VertexProperty*> RandomWalk;
  /// \brief Container for multiple random walks.
  typedef std::vector<RandomWalk> RandomWalks;
  /// \brief A sparse representation of transition probabilities between each
  /// node in the graph graph_.
  typedef Eigen::SparseMatrix<float> TransitionProbMatrix;

  /// \brief Aside from storing the random walks as a sequence of
  /// VertexProperty pointers, the RandomWalker also maps each walk to a unique
  /// integer identifier, that is used to check if two vertices share the
  /// same random walk. The value/object mapped by the key is the following
  /// struct capturing the mapped random walk, and the number of times the
  /// random walk appears for the source vertex.
  struct MappedWalk {
    const RandomWalk& random_walk_;
    int multiplicity_;

    MappedWalk(const RandomWalk& random_walk)
        : random_walk_(random_walk),
          multiplicity_(1) {
    }

    /// \brief Utility operator to increase the multiplicity of by one.
    /// \details This operator is used whenever a new random walk is inserted to
    /// the walk map and the same walk ID is already present.
    MappedWalk& operator++() { // ++MappedWalk
      ++this->multiplicity_;
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
  RandomWalker(const Graph::GraphType& graph,
               const RandomWalkerParams& params = RandomWalkerParams(),
               const int random_seed = 0);

  /**
   * \brief Access to the transition probability matrix.
   * \return A const reference to the transition probability matrix computed
   * for the graph passed to the constructor of this object.
   */
  const TransitionProbMatrix& getTransitionProbabilityMatrix() const {
    return transition_probabilities_;
  }

  /**
   * \brief Access to the generated random walks.
   * \return A const reference to the random walks generated for the vertices
   * of the graph.
   */
  const std::vector<RandomWalks>& getRandomWalks() const {
    return random_walks_;
  }

  const std::vector<WalkMap>& getMappedWalks() const {
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
    return random_walks_[vertex_index];
  }

 private:
  /// \brief Distribution to be used when picking a new random number.
  /// Needs to be mutable as it is used inside a const member function.
  mutable std::uniform_real_distribution<float> random_distribution_;
  /// \brief Random engine used to generate new random numbers.
  /// Needs to be mutable as it is used inside a const member function.
  mutable std::mt19937_64 random_engine_;

  /**
   * \brief Precomputation of transition probabilities between each neighbor
   * node in the graph graph_
   * \details This function computes transition probabilities between
   * vertices in the graph without any sort of weight between nodes. This
   * means that the probability of going from node i to any of its adjacent
   * vertices is the same independently of the node labels.
   */
  void precomputeUniformTransitionProbabilities();

  /**
   * \brief Precomputation of transition probabilities between each neighbor
   * node in the graph graph_ avoiding transitions between nodes with same
   * semantic label.
   * \details This function computes transition probabilities between
   * vertices in the graph without any sort of weight between nodes.
   * Nevertheless the probability of going from a node to one of its
   * neighbors is only non-zero if their semantic labels are different.
   */
  void precomputeAvoidingTransitionProbabilities();

  static const int computeRandomWalkKey(const RandomWalk& random_walk);

 protected:
  /**
   * \brief Generated num_walks random walks of length walk_length for each
   * node contained in the graph graph_. The length and number of random
   * walks generated for each graph vertex is determined by the parameters
   * passed to the constructor.
   */
  void generateRandomWalks();

  /**
   * \brief Samples a new vertex from the set of neighbors of the current
   * vertex based on the transition probability matrix.
   * \param current_vertex_index Current vertex index of the random walk.
   * \return A new vertex being neighbor of current, sampled randomly
   * following the transition probabilities contained in
   * transition_probabilities_
   */
  const Graph::VertexDescriptor nextVertex(const int current_vertex_index)
  const;

  /// \brief All random walks generated by this RandomWalker instance are
  /// based on the graph graph_.
  const Graph::GraphType& graph_;

  /// \brief Parameters used to generate the random walks.
  const RandomWalkerParams params_;

  /// \brief Transition probability matrix which contains a row for each node
  /// in the graph graph_. The transition probability between node i and node
  /// j is contained in transition_probabilities_.coeff(i,j)
  TransitionProbMatrix transition_probabilities_;

  /// \brief Container filled up with the random walks. The i-th element of
  /// random_walks_ is the set of random walks starting from the i-th vertex
  /// of the graph graph_.
  std::vector<RandomWalks> random_walks_;

  std::vector<WalkMap> mapped_walks_;

};

}

#undef DEFAULT_NUM_RANDOM_WALKS
#undef DEFAULT_RANDOM_WALK_LENGTH

#endif //X_VIEW_RANDOM_WALKER_H

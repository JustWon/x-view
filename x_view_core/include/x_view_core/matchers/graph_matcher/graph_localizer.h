#ifndef X_VIEW_GRAPH_LOCALIZER_H
#define X_VIEW_GRAPH_LOCALIZER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/graph_matcher.h>
#include <x_view_core/x_view_types.h>

namespace x_view {

class GraphLocalizer {

 public:
  /**
   * \brief Graph localizer constructor.
   * \param prior_noise Noise model to assume during localization for the
   * priors associated to the observed landmarks, i.e. their 3D location.
   */
  GraphLocalizer(const double prior_noise = 0.0);

  /**
   * \brief Adds an observation to the list of factors to consider during
   * optimization.
   * \param vertex_property Vertex property associated to the observation.
   * \param distance Distance measurement between unknown robot position
   * and observed vertex.
   * \param evidence Evidence that the observation being added is a true match.
   */
  void addObservation(const VertexProperty& vertex_property,
                      const double distance, const double evidence = 1.0);

  /**
   * \brief Function that estimates a transformation based on the
   * result of the descriptor matching. The estimation is based on a RANSAC
   * consensus.
   * \param matching_result Const reference to the observations.
   * \param query_semantic_graph Const reference to the (local) query
   * semantic graph
   * \param database_semantic_graph Const reference to the (global) database
   * semantic graph
   * \param transformation Return value of the transformation.
   * \return Indicator if transformation estimation succeeded.
   */
  bool estimateTransformation(
      const GraphMatcher::GraphMatchingResult& matching_result,
      const Graph& query_semantic_graph, const Graph& database_semantic_graph,
      SE3* transformation);

  /**
   * \brief Localizes the robot in the world coordinate frame by optimizing
   * the nonlinear factor graph associated to the observations.
   * \return Estimated 3D position of the robot.
   */
  const Eigen::Vector3d localize() const;

 private:
  struct Observation {
    /// \brief Vertex property associated to the observation. Its 3D location
    /// is used for localization.
    VertexProperty vertex_property;
    /// \brief Observed distance from the camera to the semantic entity
    /// associated with the vertex property.
    double distance;
    /// \brief Evidence associated to the observation. This value could be
    /// associated to the vertex similarity computed during graph matching
    /// and is used in the localization procedure for the computation of the
    /// noise model associated to the observation factors.
    double evidence;
  };

  const double prior_noise_;
  std::vector<Observation> observations_;
};

}

#endif //X_VIEW_GRAPH_LOCALIZER_H

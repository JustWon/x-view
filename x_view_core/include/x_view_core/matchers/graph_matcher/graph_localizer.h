#ifndef X_VIEW_GRAPH_LOCALIZER_H
#define X_VIEW_GRAPH_LOCALIZER_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <kindr/minimal/quat-transformation-gtsam.h>

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
  GraphLocalizer(const real_t prior_noise = 0.0);

  /**
   * \brief Adds an observation to the list of factors to consider during
   * optimization.
   * \param vertex_property Vertex property associated to the observation.
   * \param distance Distance measurement between unknown robot position
   * and observed vertex.
   * \param evidence Evidence that the observation being added is a true match.
   */
  void addObservation(const VertexProperty& vertex_property,
                      const real_t distance, const real_t evidence = 1.0);

  void addPosePoseMeasurement(PoseId pose_id_a, PoseId pose_id_b);

  void addPoseVertexMeasurement(const VertexProperty& vertex_property,
                                const PoseId& pose_id,
                                const double evidence = 1.0);

  void addVertexVertexMeasurement(const VertexProperty& vertex_property_a,
                                  const VertexProperty& vertex_property_b,
                                  double similarity);

  /**
   * \brief Localizes the robot in the world coordinate frame by optimizing
   * the nonlinear factor graph associated to the observations or based on a
   * geometric estimation.
   * \param matching_result Const reference to the observations.
   * \param query_semantic_graph Const reference to the (local) query
   * semantic graph
   * \param database_semantic_graph Const reference to the (global) database
   * semantic graph
   * \param transformation Return value of the transformation.
   * \return Indicator if localization succeeded.
   */
  bool localize(const GraphMatcher::GraphMatchingResult& matching_result,
                const Graph& query_semantic_graph,
                const Graph& database_semantic_graph,
                SE3* transformation);

  gtsam::ExpressionFactor<SE3> relativeFactor(const SE3& relative_pose,
                                              int index_a,
                                              int index_b,
                                              gtsam::noiseModel::Base::shared_ptr noise_model) const;

  gtsam::ExpressionFactor<SE3> absolutePoseFactor(const SE3& pose_measurement,
                                                  int index,
                                                  gtsam::noiseModel::Base::shared_ptr noise_model) const;

  gtsam::ExpressionFactor<gtsam::Point3> relativePointFactor(const gtsam::Point3& translation,
                                                             int index_a,
                                                             int index_b,
                                                             gtsam::noiseModel::Base::shared_ptr noise_model) const;

  bool localize2(const GraphMatcher::GraphMatchingResult& matching_result,
                 const Graph& query_semantic_graph,
                 const Graph& database_semantic_graph, SE3* transformation);

 private:
  struct Observation {
    /// \brief Vertex property associated to the observation. Its 3D location
    /// is used for localization.
    VertexProperty vertex_property;
    /// \brief Observed distance from the camera to the semantic entity
    /// associated with the vertex property.
    real_t distance;
    /// \brief Evidence associated to the observation. This value could be
    /// associated to the vertex similarity computed during graph matching
    /// and is used in the localization procedure for the computation of the
    /// noise model associated to the observation factors.
    real_t evidence;
  };

  struct PoseVertexMeasurement {
    /// \brief Vertex property associated to the observation. Its 3D location
    /// is used for localization.
    VertexProperty vertex_property;
    /// \brief Pose of the observer that encounters the vertex_property.
    PoseId observer_pose;
    /// \brief Evidence associated to the observation. This value could be
    /// associated to the vertex similarity computed during graph matching
    /// and is used in the localization procedure for the computation of the
    /// noise model associated to the observation factors.
    double evidence;
  };

  struct PosePoseMeasurement {
    /// \brief First pose of the robot.
    PoseId pose_a;
    /// \brief Second pose of the robot.
    PoseId pose_b;
  };

  struct VertexVertexMeasurement {
    /// \brief First matched vertex.
    VertexProperty vertex_a;
    /// \brief Second matched vertex.
    VertexProperty vertex_b;
    /// \brief Similarity of matched vertices.
    double evidence;
  };

  const real_t prior_noise_;
  std::vector<Observation> observations_;
  std::vector<PoseVertexMeasurement> pose_vertex_measurements_;
  std::vector<PosePoseMeasurement> pose_pose_measurements_;
  std::vector<VertexVertexMeasurement> vertex_vertex_measurements_;
};

}

#endif //X_VIEW_GRAPH_LOCALIZER_H

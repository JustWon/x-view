#ifndef X_VIEW_GRAPH_LOCALIZER_H
#define X_VIEW_GRAPH_LOCALIZER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/x_view_types.h>

namespace x_view {

class GraphLocalizer {

 public:
  /**
   * \brief Graph localizer constructor.
   * \param prior_noise Noise model to assume during localization for the
   * priors associated to the observed landmarks, i.e. their 3D location.
   * \param range_noise Noise model to assume during localization for the
   * range measurements to the observed landmarks.
   */
  GraphLocalizer(const double prior_noise = 0.0,
                 const double range_noise = 0.1);

  /**
   * \brief Adds an observation to the list of factors to consider during
   * optimization.
   * \param vertex_property Vertex property associated to the observation.
   * \param observation Distance measurement between unknown robot position
   * and observed vertex.
   */
  void addObservation(const VertexProperty& vertex_property,
                      const double observation);

  /**
   * \brief Localizes the robot in the world coordinate frame by optimizing
   * the nonlinear factor graph associated to the observations.
   * \return Estimated 3D position of the robot.
   */
  const Eigen::Vector3d localize() const;

 private:
  struct Observation {
    VertexProperty vertex_property;
    double measurement;
  };

  const double prior_noise_;
  const double range_noise_;

  std::vector<Observation> observations_;
};

}

#endif //X_VIEW_GRAPH_LOCALIZER_H

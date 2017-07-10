#include <x_view_core/matchers/graph_matcher/graph_localizer.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

namespace x_view {

GraphLocalizer::GraphLocalizer()
    : observations_(0) {
}

const Eigen::Vector3d GraphLocalizer::localize() const {

  // Create a factor graph container.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initials;

  // Add all observations to the factor graph adding a prior on their
  // position with a small noise (we want their position to stay fix during
  // optimization).
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Zero());


  for(int i = 0; i < observations_.size(); ++i) {
    gtsam::Point3 location(observations_[i].vertex_property.location_3d);

    graph.add(gtsam::PriorFactor<gtsam::Point3>(
        gtsam::Symbol('x', i),location, prior_noise));
    initials.insert(gtsam::Symbol('x', i),location);
  }

  // Add observation measurements to the factor graph.
  gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Ones() * 0.1);

  for(int i = 0; i < observations_.size(); ++i) {

    graph.add(gtsam::BetweenFactor<gtsam::Point3>(
        gtsam::Symbol('r', 0), gtsam::Symbol('x', i),
        gtsam::Point3(observations_[i].measurement), observation_noise));
  }

  // Compute initial guess for robot position.
  gtsam::Vector3 initial_robot_position = gtsam::Vector3::Zero();
  for(int i = 0; i < observations_.size(); ++i) {
    initial_robot_position += observations_[i].vertex_property.location_3d;
  }
  initial_robot_position /= observations_.size();
  initials.insert(gtsam::Symbol('r', 0), gtsam::Point3(initial_robot_position));

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initials);
  gtsam::Values results = optimizer.optimize();

  const gtsam::Point3 robot_position =
      results.at<gtsam::Point3>(gtsam::Symbol ('r', 0));

  return  Eigen::Vector3d(robot_position[0], robot_position[1],
                          robot_position[2]);
}

void GraphLocalizer::addObservation(const VertexProperty& vertex_property,
                                    const Eigen::Vector3d observation) {
  observations_.push_back(Observation{vertex_property, observation});
}

}

#include <x_view_core/matchers/graph_matcher/graph_localizer.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>

namespace x_view {

GraphLocalizer::GraphLocalizer(const double prior_noise)
    : prior_noise_(prior_noise),
      observations_(0) {
}

const Eigen::Vector3d GraphLocalizer::localize() const {

  if(observations_.size() < 4) {
    LOG(ERROR) << "Localization only works correctly with N>=4 "
        "observations! Given observations: " << observations_.size() << ".";
  }

  if(observations_.size() < 7) {
    LOG(WARNING) << "Localization result might be inaccurate due to lack of "
        "data: using " << observations_.size() << " observations.";
  }

  LOG(INFO) << "Localizing robot with " << observations_.size() << " semantic"
      " observations.";

  // Create a factor graph container.
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initials;

  // Add all observations to the factor graph adding a prior on their
  // position with a small noise (we want their position to stay fix during
  // optimization).
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3::Ones() * prior_noise_);


  for(int i = 0; i < observations_.size(); ++i) {
    gtsam::Point3 location(observations_[i].vertex_property.location_3d);

    graph.add(gtsam::PriorFactor<gtsam::Point3>(
        gtsam::Symbol('x', i), location, prior_noise));
    initials.insert(gtsam::Symbol('x', i), location);
  }

  for(int i = 0; i < observations_.size(); ++i) {
    // Add observation distance to the factor graph.
    gtsam::noiseModel::Diagonal::shared_ptr observation_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector1::Ones() * (
                1./(0.001 + observations_[i].evidence) - 1.0));
    graph.add(gtsam::RangeFactor<gtsam::Point3>(
        gtsam::Symbol('r', 0), gtsam::Symbol('x', i),
        observations_[i].distance, observation_noise));
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

  const Eigen::Vector3d computed_robot_pose(robot_position[0],
                                      robot_position[1],
                                      robot_position[2]);
  return computed_robot_pose;
}

void GraphLocalizer::addObservation(const VertexProperty& vertex_property,
                                    const double distance,
                                    const double evidence) {
  observations_.push_back(Observation{vertex_property, distance, evidence});
}

}

#include "test_localization.h"

#include <x_view_core/x_view_types.h>
#include <x_view_core/matchers/graph_matcher/graph_localizer.h>

#include <random>

namespace x_view_test {

void testLocalization(const int num_tests, const int num_observations,
                      const int seed) {

  std::mt19937 rng(seed);
  const double bound = 30;
  std::uniform_real_distribution<double> dist(-bound, bound);

  for(int t = 0; t < num_tests; ++t) {

    // Real robot position.
    Eigen::Vector3d robot_position;
    robot_position << dist(rng), dist(rng), dist(rng);

    x_view::GraphLocalizer graph_localizer;

    // Add the observations to the graph_localizer.
    for(int o = 0; o < num_observations; ++o) {
      x_view::VertexProperty vertex;
      vertex.location_3d << dist(rng), dist(rng), dist(rng);
      Eigen::Vector3d observation = vertex.location_3d - robot_position;

      graph_localizer.addObservation(vertex, observation);
    }

    const Eigen::Vector3d res = graph_localizer.localize();

    const Eigen::Vector3d diff = res - robot_position;
    const double dist = diff.dot(diff);

    CHECK_NEAR(dist, 0.0, 0.01);
  }
}

void testLocalizationUnderRotation() {

  x_view::GraphLocalizer graph_localizer;

  // Robot position expressed in world coordinates.
  Eigen::Vector3d robot_position;
  robot_position << 1, 1, 1;

  Eigen::Matrix3d rot;
  rot << 0, 0, 1,
      1, 0, 0,
      0, 1, 0;

  Eigen::Matrix3d ri = rot.inverse();

  x_view::VertexProperty vertex1;
  // World coordinates of observation.
  vertex1.location_3d << 3, 1, 2;
  // Observation expressed in robot frame.
  Eigen::Vector3d observation1 = ri*(vertex1.location_3d - robot_position);
  std::cout << "Observation1: " << observation1 << std::endl;
  graph_localizer.addObservation(vertex1, observation1);

  x_view::VertexProperty vertex2;
  // World coordinates of observation.
  vertex2.location_3d << -2, 0, -4;
  // Observation expressed in robot frame.
  Eigen::Vector3d observation2 = ri*(vertex2.location_3d - robot_position);
  std::cout << "Observation2: " << observation2 << std::endl;
  graph_localizer.addObservation(vertex2, observation2);

  x_view::VertexProperty vertex3;
  // World coordinates of observation.
  vertex3.location_3d << 0, 2, 3;
  // Observation expressed in robot frame.
  Eigen::Vector3d observation3 = ri*(vertex3.location_3d - robot_position);
  std::cout << "Observation3: " << observation3 << std::endl;
  graph_localizer.addObservation(vertex3, observation3);

  std::cout << "Localized robot: " << graph_localizer.localize() << std::endl;


}

}



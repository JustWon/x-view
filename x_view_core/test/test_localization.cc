#include "test_localization.h"

#include <x_view_core/x_view_tools.h>
#include <x_view_core/matchers/graph_matcher/graph_localizer.h>

#include <random>

namespace x_view_test {


void testLocalization(const int num_tests, const int num_observations,
                      const int seed) {

  // Initialize the random number generator.
  std::mt19937 rng(seed);
  // Set the bounding box for random coordinate generation.
  const double bound = 30;
  std::uniform_real_distribution<double> dist(-bound, bound);

  // Test tolerance for distance between real robot pose and estimated pose.
  const double tol = 0.01;

  for(int t = 0; t < num_tests; ++t) {

    // Real robot position.
    Eigen::Vector3d robot_position;
    robot_position << dist(rng), dist(rng), dist(rng);

    x_view::GraphLocalizer graph_localizer;

    Eigen::Matrix3d rot = x_view::randomRotationMatrix(rng);

    // Add the observations to the graph_localizer.
    for(int o = 0; o < num_observations; ++o) {
      x_view::VertexProperty vertex;
      vertex.location_3d << dist(rng), dist(rng), dist(rng);
      double observation =
          (rot*(vertex.location_3d - robot_position)).norm();

      graph_localizer.addObservation(vertex, observation);
    }

    // Localize the robot via its observations and check if it fulfills the
    // test requirements.
    const Eigen::Vector3d res = graph_localizer.localize();
    const double d = (res - robot_position).norm();

    CHECK_NEAR(d, 0.0, tol);
  }
}

}



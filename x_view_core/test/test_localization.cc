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
      double distance =
          (rot*(vertex.location_3d - robot_position)).norm();

      double evidence = 1.0;

      graph_localizer.addObservation(vertex, distance, 0.3);
    }

    // Localize the robot via its observations and check if it fulfills the
    // test requirements.
    const Eigen::Vector3d res = graph_localizer.localize();
    const double d = (res - robot_position).norm();

    CHECK_NEAR(d, 0.0, tol);
  }
}

void testEvidence(const int seed) {
  Eigen::Vector3d robot_position;
  robot_position << 0,0,0;


#define ADD_OBSERVATION(x, y, z, deviation, evidence) \
  { \
     x_view::VertexProperty vertex; \
     vertex.location_3d << x, y, z; \
     const double real_distance = \
               (vertex.location_3d - robot_position).norm(); \
     const double d = real_distance * (1.0 + (deviation)); \
     const double e = (evidence); \
     graph_localizer.addObservation(vertex, d, e); \
  }

  for(double deviation = 0; deviation < 0.15; deviation += 0.025) {
    // Initialize the random number generator.
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dev(-deviation, deviation);
    std::uniform_real_distribution<double> evid(1.0 - 5.0 * deviation, 1);

    x_view::GraphLocalizer graph_localizer;

    // Add some real observation with large evidence
    ADD_OBSERVATION(1, 0, 0, dev(rng), evid(rng));
    ADD_OBSERVATION(0, 1, 0, dev(rng), evid(rng));
    ADD_OBSERVATION(0, 0, 1, dev(rng), evid(rng));
    ADD_OBSERVATION(1, 1, 0, dev(rng), evid(rng));
    ADD_OBSERVATION(-1, 0, -1, dev(rng), evid(rng));
    ADD_OBSERVATION(0, 1, 1, dev(rng), evid(rng));
    ADD_OBSERVATION(1, 1, 1, dev(rng), evid(rng));
    // Add an outlier observation with small evidence
    ADD_OBSERVATION(1, -1, -1, 10, 0.1 * (1 - deviation));

    Eigen::Vector3d res = graph_localizer.localize();
    const double dist = (res - robot_position).norm();
    CHECK_NEAR(dist, 0.0, 0.2);
  }


#undef ADD_OBSERVATION

}

}



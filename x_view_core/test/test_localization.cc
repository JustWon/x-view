#include "test_localization.h"

#include <x_view_core/matchers/graph_matcher/graph_localizer.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>

#include <random>

namespace x_view_test {


void testLocalization(const int num_tests, const int num_observations,
                      const int seed) {

  // Initialize the random number generator.
  std::mt19937 rng(seed);
  // Set the bounding box for random coordinate generation.
  const x_view::real_t bound = 30.0;
  std::uniform_real_distribution<x_view::real_t> distr(-bound, bound);

  // Test tolerance for distance between real robot pose and estimated pose.
  const x_view::real_t tol = 0.01;

  for(int t = 0; t < num_tests; ++t) {

    // Real robot position.
    x_view::Vector3r robot_position;
    robot_position << distr(rng), distr(rng), distr(rng);

    x_view::GraphLocalizer graph_localizer;

    x_view::Matrix3r rot = x_view::randomRotationMatrix(rng);

    // Add the observations to the graph_localizer.
    x_view::SE3 robot_pose;
    robot_pose.setIdentity();
    robot_pose.getPosition() = robot_position.cast<double>();
    x_view::PoseId pose_id;
    pose_id.id = x_view::KeyGenerator::getNextKey();
    pose_id.pose = robot_pose;
    for(int o = 0; o < num_observations; ++o) {
      x_view::VertexProperty vertex, vertex2;
      vertex.location_3d << distr(rng), distr(rng), distr(rng);
      vertex.observers.push_back(pose_id);
      vertex.index = x_view::KeyGenerator::getNextKey();
      vertex2 = vertex;
      vertex2.index = x_view::KeyGenerator::getNextKey();
      x_view::real_t distance =
          (rot*(vertex.location_3d - robot_position)).norm();

      x_view::real_t evidence = 0.3;
      graph_localizer.addPoseVertexMeasurement(vertex, pose_id, evidence);
      graph_localizer.addVertexVertexMeasurement(vertex, vertex2, evidence);
    }

    // Localize the robot via its observations and check if it fulfills the
    // test requirements.
    x_view::SE3 transformation;
    x_view::GraphMatcher::GraphMatchingResult matching_result;
    x_view::Graph query_graph;
    x_view::Graph database_graph;
    graph_localizer.localize2(matching_result, query_graph, database_graph,
                             &transformation);
    const x_view::real_t d =
        (transformation.getPosition().cast<x_view::real_t>() - robot_position)
            .norm();

    CHECK_NEAR(d, 0.0, tol);
  }
}

void testEvidence(const int seed) {
  x_view::Vector3r robot_position;
  robot_position << 0.0, 0.0, 0.0;


#define ADD_OBSERVATION(x, y, z, pose_id, deviation, evidence) \
  { \
     x_view::VertexProperty vertex, vertex2; \
     vertex.location_3d << x, y, z; \
     vertex.index = x_view::KeyGenerator::getNextKey(); \
     vertex.observers.push_back(pose_id); \
     vertex2 = vertex; \
     vertex2.index = x_view::KeyGenerator::getNextKey(); \
     const x_view::real_t real_distance = \
               (vertex.location_3d - robot_pose.getPosition() \
                .cast<x_view::real_t>()).norm(); \
     const x_view::real_t e = (evidence); \
     graph_localizer.addPoseVertexMeasurement(vertex, pose_id, evidence); \
     graph_localizer.addVertexVertexMeasurement(vertex, vertex2, evidence); \
  }

  for(x_view::real_t deviation = 0; deviation < 0.15; deviation += 0.025) {
    // Initialize the random number generator.
    std::mt19937 rng(seed);
    std::uniform_real_distribution<x_view::real_t> dev(-deviation, deviation);
    std::uniform_real_distribution<x_view::real_t> evid(1.0 - 5.0 * deviation, 1);
    x_view::SE3 robot_pose;
    robot_pose.setIdentity();
    x_view::PoseId pose_id;
    pose_id.id = x_view::KeyGenerator::getNextKey();
    pose_id.pose = robot_pose;

    x_view::GraphLocalizer graph_localizer;

    // Add some real observation with large evidence
    ADD_OBSERVATION(1.0, 0.0, 0.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(0.0, 1.0, 0.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(0.0, 0.0, 1.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(1.0, 1.0, 0.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(-1.0, 0.0, -1.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(0.0, 1.0, 1.0, pose_id, dev(rng), evid(rng));
    ADD_OBSERVATION(1.0, 1.0, 1.0, pose_id, dev(rng), evid(rng));
    // Add an outlier observation with small evidence
    ADD_OBSERVATION(1.0, -1.0, -1.0, pose_id, 10.0, 0.1 * (1.0 - deviation));

    x_view::SE3 transformation;
    x_view::GraphMatcher::GraphMatchingResult matching_result;
    x_view::Graph query_graph;
    x_view::Graph database_graph;
    graph_localizer.localize2(matching_result, query_graph, database_graph,
                             &transformation);
    const x_view::real_t dist = (transformation.getPosition()
        .cast<x_view::real_t>() - robot_position).norm();
    CHECK_NEAR(dist, 0.0, 0.2);
  }


#undef ADD_OBSERVATION

}

}



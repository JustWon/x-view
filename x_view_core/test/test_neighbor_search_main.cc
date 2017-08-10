#include <gtest/gtest.h>

#include "test_neighbor_search.h"

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_neighbor_search) {

  // Tests the neighbor search procedure used in X-View.
  LOG(INFO) << "\n\n====Testing neighbor search====";

  x_view::MatrixXr points;
  const uint64_t N = 10;
  const uint64_t D = 3;
  const x_view::real_t lower_bound = 0;
  const x_view::real_t upper_bound = 9;
  generatePointsOnLine(N, D, lower_bound, upper_bound, &points);

  const x_view::Vector3r query_point(4.4, 0, 0);
  std::vector<int> expected_indices = {
      4, 5, 3, 6
  };
  testKNN(points, query_point, expected_indices);
}





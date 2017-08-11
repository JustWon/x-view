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

  const x_view::Vector3r query_point(4.4, 2.0, -1.2);
  std::vector<int> expected_indices = {
      4, 5, 3, 6
  };
  testKNN(points, query_point, expected_indices);


  const uint64_t K = 5;
  x_view::MatrixXr pointcloud, query_points;
  std::uniform_real_distribution<x_view::real_t> distr(0, 1);
  std::mt19937 rng(0);
  generateRandomPoints(5, 3, distr, rng, &pointcloud);
  generateRandomPoints(5, 3, distr, rng, &query_points);
  query_points = pointcloud;

  NNSearch* pairwise_distance = NNSearch::createKDTreeLinearHeap(pointcloud, 3);

  x_view::MatrixXr distances(K, query_points.cols());
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> indices(K, query_points.cols());

  const x_view::real_t epsilon = 0;
  pairwise_distance->knn(query_points, indices, distances, K, epsilon,
                         NNSearch::SORT_RESULTS | NNSearch::ALLOW_SELF_MATCH);

  std::cout << "Point1:\n" << pointcloud
            <<";\n\nPoint2:\n" <<query_points <<std::endl;

  std::cout << "Indices:\n" << indices
            << ";\n\nDistances:\n" << distances <<  std::endl;
}





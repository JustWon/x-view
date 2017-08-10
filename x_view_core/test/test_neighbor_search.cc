#include "test_neighbor_search.h"

#include <glog/logging.h>


namespace x_view_test {


void generateRandomPoints(const uint64_t N, const uint64_t D,
                          std::uniform_real_distribution<x_view::real_t>& dist,
                          std::mt19937& rng,
                          x_view::MatrixXr* points) {

  CHECK_NOTNULL(points);

  points->resize(D, N);
  for(uint64_t j = 0; j < N; ++j)  {
    for(uint64_t i = 0; i < D; ++i)
    points->operator()(i, j) = dist(rng);
  }
}

void generatePointsOnLine(const uint64_t N, const uint64_t D,
                          const x_view::real_t lower_bound,
                          const x_view::real_t upper_bound,
                          x_view::MatrixXr* points) {
  CHECK_NOTNULL(points);

  points->resize(D, N);
  const x_view::real_t dx = (upper_bound - lower_bound) / (N - 1);
  x_view::real_t pos = lower_bound;
  for(uint64_t j = 0; j < N; ++j) {
    points->operator()(0, j) = pos;
    pos += dx;
    for(uint64_t i = 1; i < D; ++i) {
      points->operator()(i, j) = 0;
    }
  }
}

void testKNN(const x_view::MatrixXr& points, const x_view::Vector3r& query,
             const std::vector<uint64_t>& expected_indices) {

  CHECK(points.rows() == query.rows());

  const uint64_t D = points.rows();
  const uint64_t K = expected_indices.size();

  NNSearch* knn_tree =
      Nabo::NearestNeighbourSearch::createKDTreeLinearHeap(points, D);

  IndexVector indices;
  DistanceVector distances;

  knn_tree->knn(query, indices, distances, K, NNSearch::SORT_RESULTS);

  for(int i = 0; i < K; ++i) {
    CHECK(expected_indices[i] == indices(i));
  }

}

}

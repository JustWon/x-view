#include "test_neighbor_search.h"

#include <x_view_core/landmarks/graph_landmark/blob.h>

#include <chrono>

namespace x_view_test {

void testNabo2D() {

  x_view::MatrixXr points;
  const uint64_t N = 10;
  const uint64_t D = 2;
  const x_view::real_t lower_bound = 0;
  const x_view::real_t upper_bound = 9;
  generatePointsOnLine(N, D, lower_bound, upper_bound, &points);

  const Vector2r query_point(4.4, 2.0);
  std::vector<int> expected_indices = {
      4, 5, 3, 6
  };

  testKNN(points, query_point, expected_indices);
}

void generatePointsOnLine(const uint64_t N, const uint64_t D,
                          const x_view::real_t lower_bound,
                          const x_view::real_t upper_bound,
                          x_view::MatrixXr* points) {
  CHECK_NOTNULL(points);

  points->resize(D, N);
  const x_view::real_t dx = (upper_bound - lower_bound) / (N - 1);
  x_view::real_t pos = lower_bound;
  for (uint64_t j = 0; j < N; ++j) {
    points->operator()(0, j) = pos;
    pos += dx;
    for (uint64_t i = 1; i < D; ++i) {
      points->operator()(i, j) = 0;
    }
  }
}

void testKNN(const x_view::MatrixXr& points, const Vector2r& query,
             const std::vector<int>& expected_indices) {

  CHECK(points.rows() == query.rows());

  const uint64_t D = points.rows();
  const uint64_t K = expected_indices.size();

  NNSearch* knn_tree = NNSearch::createKDTreeLinearHeap(points, D);

  IndexVector indices;
  DistanceVector distances;

  const x_view::real_t epsilon = 0.0;
  knn_tree->knn(query, indices, distances, K, epsilon,
                NNSearch::SORT_RESULTS | NNSearch::ALLOW_SELF_MATCH);

  for (int i = 0; i < K; ++i) {
    CHECK(expected_indices[i] == indices(i));
  }

}

void testBlobNeighborPerformance() {
  // Generate two non-neighbor blobs.
  x_view::Blob blob_i, blob_j;

  enum DIRECTION {
    UP = 0,
    RIGHT = 1,
    DOWN = 2,
    LEFT = 3
  };

  std::map<DIRECTION, cv::Point2i> step_map = {
      {UP, cv::Point2i(0, 1)},
      {RIGHT, cv::Point2i(1, 0)},
      {DOWN, cv::Point2i(0, -1)},
      {LEFT, cv::Point2i(-1, 0)},
  };

  std::uniform_int_distribution<int> step_dist(0, 4);
  std::mt19937 rng(2);

  auto nextStep = [&]() -> cv::Point2i {
    return step_map[static_cast<DIRECTION>(step_dist(rng))];
  };

  const uint64_t num_external_pixels = 1000;
  const int distance_threshold = 18;

  cv::Point2i start_i(0, 0);
  std::vector<cv::Point2i> external_contours_i;
  external_contours_i.push_back(start_i);
  for (int i = 0; i < num_external_pixels; ++i) {
    external_contours_i.push_back(external_contours_i.back() + nextStep());
  }

  cv::Point2i start_j(20, 20);
  std::vector<cv::Point2i> external_contours_j;
  external_contours_j.push_back(start_j);
  for (int i = 0; i < num_external_pixels; ++i) {
    external_contours_j.push_back(external_contours_j.back() + nextStep());
  }

  blob_i.external_contour_pixels = external_contours_i;
  blob_j.external_contour_pixels = external_contours_j;

  std::chrono::duration<x_view::real_t, std::ratio<1, 1000>>
      with_libnabo_duration, without_libnabo_duration;

  const uint64_t repetitions = 1000;
  {
    const bool use_libnabo = true;
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repetitions; ++i) {
      if (i % 2)
        x_view::Blob::areNeighbors(blob_i, blob_j, distance_threshold,
                                   use_libnabo);
      else
        x_view::Blob::areNeighbors(blob_j, blob_i, distance_threshold,
                                   use_libnabo);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    with_libnabo_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
  }
  {
    const bool use_libnabo = false;
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repetitions; ++i) {
      if (i % 2)
        x_view::Blob::areNeighbors(blob_i, blob_j, distance_threshold,
                                   use_libnabo);
      else
        x_view::Blob::areNeighbors(blob_j, blob_i, distance_threshold,
                                   use_libnabo);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    without_libnabo_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
  }
  if (with_libnabo_duration < without_libnabo_duration)
    std::cout << "Libnabo is more performing than bruteforce approach:\n";
  else
    std::cout << "Libnabo is less performing than bruteforce approach:\n";
  std::cout << "\tLibnabo: " << with_libnabo_duration.count() << " ms.\n"
            << "\tWithout libnabo: " << without_libnabo_duration.count()
            << " ms." << std::endl;
}

}

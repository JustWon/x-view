#ifndef X_VIEW_TEST_NEIGHBOR_SEARCH_H
#define X_VIEW_TEST_NEIGHBOR_SEARCH_H

#include <x_view_core/x_view_types.h>

#include <nabo/nabo.h>

#include <random>

namespace x_view_test {

typedef Eigen::VectorXi IndexVector;

typedef Eigen::Matrix<x_view::real_t, Eigen::Dynamic, 1> DistanceVector;

typedef Nabo::NearestNeighbourSearch<x_view::real_t> NNSearch;

void generateRandomPoints(const uint64_t N, const uint64_t D,
                          std::uniform_real_distribution<x_view::real_t>& dist,
                          std::mt19937& rng,
                          x_view::MatrixXr* points);

void generatePointsOnLine(const uint64_t N, const uint64_t D,
                          const x_view::real_t lower_bound,
                          const x_view::real_t upper_bound,
                          x_view::MatrixXr* points);

void testKNN(const x_view::MatrixXr& points, const x_view::Vector3r& query,
             const std::vector<uint64_t>& expected_indices);

}

#endif //X_VIEW_TEST_NEIGHBOR_SEARCH_H

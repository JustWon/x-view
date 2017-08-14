#ifndef X_VIEW_TEST_NEIGHBOR_SEARCH_H
#define X_VIEW_TEST_NEIGHBOR_SEARCH_H

#include <x_view_core/x_view_types.h>

#include <nabo/nabo.h>

#include <random>

namespace x_view_test {

typedef Eigen::VectorXi IndexVector;
typedef Eigen::Matrix<x_view::real_t, Eigen::Dynamic, 1> DistanceVector;
typedef Eigen::Matrix<x_view::real_t, 2, 1> Vector2r;

typedef Nabo::NearestNeighbourSearch<x_view::real_t> NNSearch;

void testNabo2D();

void generatePointsOnLine(const uint64_t N, const uint64_t D,
                          const x_view::real_t lower_bound,
                          const x_view::real_t upper_bound,
                          x_view::MatrixXr* points);

void testKNN(const x_view::MatrixXr& points, const Vector2r& query,
             const std::vector<int>& expected_indices);


void testBlobNeighborPerformance();

}

#endif //X_VIEW_TEST_NEIGHBOR_SEARCH_H

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

/**
 * \brief Tests the functionality of libnabo by checking if the computed
 * nearest neighbors correspond to a set of manually selected ones.
 */
void testNabo2D();

/**
 * \brief Generates N D-dimensional points on a 1-dimensional line aligned
 * with the first coordinate between lower_bound and upper_bound.
 * \param N Numbers of points to generate.
 * \param D Dimensionality of the points being generated.
 * \param lower_bound Scalar indicating the smallest value associated to the
 * first coordinate.
 * \param upper_bound Scalar indicating the largest value associated to the
 * first coordinate.
 * \param points Matrix of size DxN filled up with the generated points.
 */
void generatePointsOnLine(const uint64_t N, const uint64_t D,
                          const x_view::real_t lower_bound,
                          const x_view::real_t upper_bound,
                          x_view::MatrixXr* points);

/**
 * \brief Tests the results provided by libnabo.
 * \param points Pointcloud over which libnabo builds a KD-tree.
 * \param query Query point for which libnabo searches nearest neighbors in
 * the pointcloud.
 * \param expected_indices Ordered vector of indices in pointcloud specifying
 * closest neighbors of query point.
 */
void testKNN(const x_view::MatrixXr& points, const Vector2r& query,
             const std::vector<int>& expected_indices);


/**
 * \brief Builds two x_view::Blob instances with random external_pixels and
 * tests the perdormance of the x_view::Blob::areNeighors() function for
 * using the functionalities offered by libnabo and the bruteforce approach.
 */
void testBlobNeighborPerformance();

}

#endif //X_VIEW_TEST_NEIGHBOR_SEARCH_H

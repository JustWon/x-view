#ifndef X_VIEW_TEST_GRAPH_LANDMARK_IMPL_H
#define X_VIEW_TEST_GRAPH_LANDMARK_IMPL_H

#include <gtest/gtest.h>

#include <x_view_core/x_view_types.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>

#include <opencv2/core/core.hpp>

#include <boost/progress.hpp>

using namespace x_view;

typedef std::shared_ptr<GraphLandmark> GraphLandmarkPtr;

/// \brief test the custom image
void testCustomImage();
/// \brief test the disc image
void testDiscImage();

/**
 * \brief Checks if the number of instances per class found by the
 * graphLandmarkPrt object is the same as the expected one
 * \param graphLandmarkPtr pointer to the graphLandmark
 * \param expectedInstanceCount expected number of instances per class, such
 * that 'expectedInstanceCount[i]' contains the number of expected instances
 * for class 'i'
 * \param imageName logging image name
 */
void testInstanceCount(const GraphLandmarkPtr& graphLandmarkPtr,
                       const std::vector<int>& expectedInstanceCount,
                       const std::string& imageName);

/**
 * \brief Creates a custom image of size 'desiredRows' x 'desiredCols'
 * \param desiredRows desired number of rows
 * \param desiredCols desired number of cols
 * \param image generated image
 */
void createCustomImage(const int desiredRows, const int desiredCols,
                       cv::Mat& image);

/**
 * \brief Creates a chessboard-like image of approximate size 'desiredRows' x
 * 'desiredCols' with a block size of 'block_size'. The generated image
 * represents a sets of blocks in the following order:
 * 0, 1, 2, 3, ... , N-1, 0, 1, 2, 3, ..., N-1, ...., N-1
 * 1, 2, 3, 4, ... , 0, 1, 2, 3, 4, ...
 * ...
 * where 'N' is equal to 'globalDatasetPtr->numSemanticClasses()'.
 * For this reason the final image size might not be exactly the same as the
 * one passed as parameter
 * \param desiredRows desired number of rows
 * \param desiredCols desired number of cols
 * \param block_size size of the chessboard block
 * \param image generated image
 */
void createChessBoardImage(const int desiredRows, const int desiredCols,
                           const int block_size, cv::Mat& image);

/**
 * \brief Creates an image containing a set of discs
 * \param desiredRows desired number of rows
 * \param desiredCols desired number of cols
 * \param centers vector of disc centers (cv::Point)
 * \param radii vector of disc radii
 * \param labels vector of label to associate to each disc
 * \param image generated image
 */
void createDiscImage(const int desiredRows, const int desiredCols,
                     const std::vector<cv::Point>& centers,
                     const std::vector<int> radii,
                     const std::vector<int> labels, cv::Mat& image);

#endif //X_VIEW_TEST_GRAPH_LANDMARK_IMPL_H

#ifndef X_VIEW_TEST_GRAPH_LANDMARK_H
#define X_VIEW_TEST_GRAPH_LANDMARK_H

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark.h>
#include <x_view_core/x_view_types.h>

#include <boost/progress.hpp>
#include <opencv2/core/core.hpp>

using namespace x_view;

typedef std::shared_ptr<GraphLandmark> GraphLandmarkPtr;

/// \brief test the custom image
void testCustomImage();
/// \brief test the disc image
void testDiscImage();

/**
 * \brief Counts how many pixels with each label are present in the image
 * \param image cv::Mat to be analyzed
 * \param pixel_count vector filled up with the pixel count such that
 * 'pixelCount[i]' contains the number of pixels in 'image' having label 'i'
 */
void countPixelLabelsInImage(const cv::Mat& image,
                             std::vector<int>& pixel_count);

/**
 * \brief Checks if the number of pixels for each class 'i' in 'image' is the
 * same between counting them explicitly and the one contained in the
 * graphLandmarkPointer
 * \param graphLandmarkPtr pointer to the graphLandmark
 * \param imageName logging image name
 */
void testPixelCount(const GraphLandmarkPtr& graphLandmarkPtr,
                    const std::string& imageName);

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

#endif //X_VIEW_TEST_GRAPH_LANDMARK_H
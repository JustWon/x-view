/* This test computes the graph representation of different images and tests
 * various properties associated to them.
 * Such properties could be the number of pixels associated to each semantic
 * label or the uniqueness of pixel in the blob datastructure in the
 * graphLandmark objects
 *
 * The implementation of the tests is located in 'test_graph_landmark.cc'
 */

#include "test_graph_landmark.h"

#include <gtest/gtest.h>

TEST(XViewSlamTestSuite, test_graphLandmark) {
  // set the minimum blob size to zero because for testing we don't want to
  // ignore any generated blob
  GraphLandmark::MINIMUM_BLOB_SIZE = 0;
  // don't perform dilation and erosion on the input images.
  GraphLandmark::DILATE_AND_ERODE = true;

  // test different images
  testCustomImage();
  testDiscImage();
}


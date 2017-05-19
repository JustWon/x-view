/* This test computes the graph representation of different images and tests
 * various properties associated to them.
 * Such properties could be the number of pixels associated to each semantic
 * label or the uniqueness of pixel in the blob datastructure in the
 * graphLandmark objects
 *
 * The implementation of the tests is located in 'test_graph_landmark_impl.cc'
 */

#include <gtest/gtest.h>

#include "test_graph_landmark_impl.h"

TEST(XViewSlamTestSuite, test_graphLandmark) {
  // set the minimum blob size to zero because for testing we don't want to
  // ignore any generated blob
  GraphLandmark::MINIMUM_BLOB_SIZE = 0;
  GraphLandmark::DILATE_AND_ERODE = false;

  // test different images
  testCustomImage();
  testDiscImage();
}



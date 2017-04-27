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

  // test different images
  testCustomImage();
  testChessboardImage();
  testDiscImage();
}



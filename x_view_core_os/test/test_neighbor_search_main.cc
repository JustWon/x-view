#include <gtest/gtest.h>

#include "test_neighbor_search.h"

using namespace x_view_test;

TEST(XViewSlamTestSuite, test_neighbor_search) {

  // Tests the neighbor search procedure used in X-View.
  LOG(INFO) << "\n\n====Testing neighbor search====";

  // Test if libnabo works as expected.
  testNabo2D();

  // Test if introducing libnabo and the associated datastructures offers an
  // advantage over the bruteforce neighbor computation.
  testBlobNeighborPerformance();
}





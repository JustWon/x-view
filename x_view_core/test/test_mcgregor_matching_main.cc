#include <gtest/gtest.h>

#include "test_mcgregor_matching.h"

int mc_gregor_maximal_num_edges;

TEST(XViewSlamTestSuite, test_mcgregor_matching) {

  // Run a bunch of tests for different graphs and compute matches with
  // mcgregor algorithm
  SimpleGraphsTest simple_graph_test;
  simple_graph_test.run();

  PaperGraphsTest paper_graphs_test;
  paper_graphs_test.run();
}




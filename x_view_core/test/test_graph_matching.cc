#include <gtest/gtest.h>

#include "test_graph_matching_impl.h"

int mc_gregor_maximal_num_edges;

TEST(XViewSlamTestSuite, graphMatching) {

  // Run a bunch of tests for different graphs
  SimpleGraphsTest simple_graph_test;
  simple_graph_test.run();

  PaperGraphsTest paper_graphs_test;
  paper_graphs_test.run();
}




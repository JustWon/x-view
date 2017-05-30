#include <gtest/gtest.h>

#include "test_walk_matching.h"

#include <x_view_core/datasets/abstract_dataset.h>

#include <glog/logging.h>

using namespace x_view;



TEST(XViewSlamTestSuite, test_walk_matching) {

  LOG(INFO) << "\n\n====Testing walk matching====";

  const int num_classes = 3;
  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_classes));

  const int num_nodes = 10;
  const int num_walks_per_node = 10;
  const int walk_length = 3;

  std::vector<ExampleNode> nodes;
  for (int i = 0; i < num_nodes; ++i) {
    nodes.push_back(ExampleNode(num_walks_per_node, walk_length,
                                num_classes, i));
    std::cout << nodes.back() << std::endl;
  }

}


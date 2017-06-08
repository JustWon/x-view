#include <gtest/gtest.h>

#include "test_walk_matching.h"

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark.h>

using namespace x_view;
using namespace x_view_test;

TEST(XViewSlamTestSuite, test_walk_matching) {

  LOG(INFO) << "\n\n====Testing walk matching====";

  const int num_semantic_classes = 13;
  LOG(INFO) << "Testing walk matching with " << num_semantic_classes
            << "classes.";

  global_dataset_ptr =
      std::make_shared<AbstractDataset>(AbstractDataset(num_semantic_classes));
  CHECK_NOTNULL(global_dataset_ptr.get());

  const unsigned long seed = 0;

  testChainGraph(seed);
  testRandomGraph(seed);

  // Close all windows.
  cv::destroyAllWindows();
}


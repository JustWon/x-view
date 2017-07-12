#include <gtest/gtest.h>
#include <glog/logging.h>

#include "test_common.h"
#include <x_view_core/x_view_tools.h>
#include <x_view_core/datasets/abstract_dataset.h>

x_view::ConstDatasetPtr global_dataset_ptr;

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {

  x_view::setupLogging(argv);
  LOG(INFO) << "\n================== Running X-View Tests ==================\n";

  testing::InitGoogleTest(&argc, argv);

  // Create dummy parameters used by the tests.
  x_view_test::createParameters();

  int test_succesfull = RUN_ALL_TESTS();

  x_view::finalizeLogging();

  return test_succesfull;
}
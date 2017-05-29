#include <gtest/gtest.h>
#include <glog/logging.h>

#include <x_view_core/x_view_tools.h>
#include <x_view_core/datasets/abstract_dataset.h>

x_view::ConstDatasetPrt global_dataset_ptr;

void setupLogging(char** argv) {

  const std::string& log_dir_name = x_view::getLogDirectory();

  std::vector<std::pair<const int, std::string> > log_file_names =
      {{google::INFO, "log_INFO"},
       {google::WARNING, "log_WARN"},
       {google::ERROR, "log_ERR"},
       {google::FATAL, "log_FATAL"}};

  for (const auto& level : log_file_names) {
    google::SetLogDestination(level.first,
                              (log_dir_name + level.second).c_str());

    google::SetLogSymlink(level.first, "__LAST");
  }

  // Print logs also to the console if their level is greater than
  // min_console_level;
  const int min_console_level = google::ERROR;
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(min_console_level);

#ifdef X_VIEW_DEBUG
  FLAGS_alsologtostderr = true;
#endif

  google::InitGoogleLogging(argv[0]);

  std::cout << "X-View tests are logging to <"
            << log_dir_name << ">" << std::endl;

}

void finalizeLogging() {
  google::FlushLogFiles(google::INFO);
  google::FlushLogFiles(google::WARNING);

  google::FlushLogFiles(google::ERROR);
  google::FlushLogFiles(google::FATAL);
}

/// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {

  setupLogging(argv);
  LOG(INFO) << "Running all X-View core tests";

  testing::InitGoogleTest(&argc, argv);

  int test_succesfull = RUN_ALL_TESTS();

  finalizeLogging();

  return test_succesfull;
}
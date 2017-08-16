#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_evaluation/x_view_evaluation.h>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  // Set up the evaluation.
  x_view_evaluation::EvaluationParameters evaluation_parameters;
  evaluation_parameters.timer_type =
      x_view_evaluation::EvaluationParameters::TIMER_TYPE::TIMER;
  x_view_evaluation::Evaluation evaluation(evaluation_parameters);

  const uint64_t start_frame = 0;
  const uint64_t end_frame = 15;

  // Build the semantic graph associated to the path specified in the
  // parameters passed to the iteration function.
  bag_reader.iterateBagFromTo(x_view_ros::CAMERA::FRONT,
                              start_frame, end_frame);

  // Try to localize the following views inside the previously constructed
  // semantic graph.
  x_view_ros::Pause pause;
  x_view::Statistics stats;
  const uint64_t local_graph_steps = 5;
  for(int i = start_frame; i + local_graph_steps < end_frame;) {
    if(!pause.isPaused()) {
      x_view_ros::XViewBagReader::LocationPair locations;
      bool localized = bag_reader.localizeGraph(x_view_ros::CAMERA::FRONT, i,
                                                local_graph_steps, &locations);
      if(localized) {
        std::cout << "Estimation: " << x_view::RowVector3r(locations.first) << "\n"
                  << "True: " << x_view::RowVector3r(locations.second) << std::endl;
        stats.insert((locations.second - locations.first).norm());
      } else {
        std::cout << "Localization was not effective." << std::endl;
      }
      ++i;
    }
  }
  pause.terminate();
  std::cout << "Mean localization error: " << stats.mean()
            << ".\nStandard deviation: " << stats.std() << std::endl;

  std::cout << "Evaluation timings table: " << std::endl;
  std::cout << evaluation.getTimingsTable() << std::endl;

  const auto all_timings = evaluation.getAllTimings();
  for(const auto& p : all_timings) {
    std::cout << "Timer " << p.first << ": ";
    for(const x_view::real_t t : p.second) {
      std::cout << t << ", ";
    }
    std::cout << std::endl;
  }


  x_view::finalizeLogging();

  return 0;
}

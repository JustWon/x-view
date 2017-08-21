#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>
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
  const uint64_t end_frame = 25;

  // Build the semantic graph associated to the path specified in the
  // parameters passed to the iteration function.
  bag_reader.iterateBagFromTo(x_view_ros::CAMERA::FRONT,
                              start_frame, end_frame);

  // Store the graph construction evaluation to a file.
  std::string graph_construction_folder =
      x_view::getOutputDirectory() + "graph_construction/";
  bool write_success = evaluation.writeToFolder(graph_construction_folder);
  std::cout << "Could " << (write_success ? "" : "not ")
            << "write localization evaluation results to "
            << graph_construction_folder << std::endl;

  // Extract the timer associated to the construction and store it locally.
  x_view::AbstractTimer* construction_timer;
  evaluation.time.storeTimer(&construction_timer);

  // Introduce noise to the global semantic graph by relabeling some vertices.
  const x_view::real_t vertex_relabeling_percentage = 0.0;
  const uint64_t vertex_relabeling_seed = 0;
  bag_reader.relabelGlobalGraphVertices(
     vertex_relabeling_percentage, vertex_relabeling_seed);


  // Try to localize the following views inside the previously constructed
  // semantic graph.
  x_view_ros::Pause pause;
  const uint64_t local_graph_steps = 5;
  for(int i = start_frame; i + local_graph_steps < end_frame;) {
    if(!pause.isPaused()) {
      x_view::LocalizationPair locations;
      bool localized = bag_reader.localizeGraph(x_view_ros::CAMERA::FRONT, i,
                                                local_graph_steps, &locations);
      if(localized) {
        std::cout << "Estimation: \n"
                  << x_view::formatSE3(locations.estimated_pose, "\t") << "\n"
                  << "True: \n"
                  << x_view::formatSE3(locations.true_pose, "\t") << std::endl;

        evaluation.localization.addLocalization("LocalizationEstimation",
                                                locations);
      } else {
        std::cout << "Localization was not effective." << std::endl;
      }
      ++i;
    }
  }
  pause.terminate();

  // Store the localization evaluation to a file.
  std::string graph_localization_folder =
      x_view::getOutputDirectory() + "graph_localization/";
  write_success = evaluation.writeToFolder(graph_localization_folder);
  std::cout << "Could " << (write_success ? "" : "not ")
            << "write localization evaluation results to "
            << graph_localization_folder << std::endl;

  x_view::finalizeLogging();

  return 0;
}

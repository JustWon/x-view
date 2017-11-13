#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/x_view_locator.h>
#include <x_view_core/x_view_tools.h>
#include <x_view_core/x_view_types.h>
#include <x_view_evaluation/x_view_evaluation.h>
#include <x_view_evaluation/path.h>


struct BagNodeParameters {

  BagNodeParameters() {
    start_frame = 0;
    end_frame = 200;

    pause_active = false;
    timer_type = x_view_evaluation::EvaluationParameters::TIMER_TYPE::TIMER;

    relabeling_percentage = static_cast<x_view::real_t>(0.0);

    graph_construction_camera = x_view_ros::CAMERA::FRONT;
    localization_camera = x_view_ros::CAMERA::FRONT;
    local_graph_steps = 5;

    run_name = "Example_run";
  }

  int start_frame;
  int end_frame;

  bool pause_active;
  x_view_evaluation::EvaluationParameters::TIMER_TYPE timer_type;

  x_view::real_t relabeling_percentage;

  x_view_ros::CAMERA graph_construction_camera;
  x_view_ros::CAMERA localization_camera;
  int local_graph_steps;

  std::string run_name;
};

void loadBagNodeParameters(const ros::NodeHandle& nh,
                           BagNodeParameters* params) {

  int start_frame;
  if (nh.getParam("/XViewBagReader/start_frame", start_frame)) {
    params->start_frame = start_frame;
  }

  int end_frame;
  if (nh.getParam("/XViewBagReader/end_frame", end_frame)) {
    params->end_frame = end_frame;
  }

  bool pause_active;
  if (nh.getParam("/XViewBagReader/pause_active", pause_active)) {
    params->pause_active = pause_active;
  }

  std::string timer_type;
  if (nh.getParam("/XViewBagReader/timer_type", timer_type)) {
    if(timer_type == "TIMER")
      params->timer_type = x_view_evaluation::EvaluationParameters::TIMER;
    else if(timer_type == "NULL_TIMER")
      params->timer_type = x_view_evaluation::EvaluationParameters::NULL_TIMER;
    else
      LOG(ERROR) << "Unrecognized timer type <" << timer_type << ".";
  }

  x_view::real_t relabeling_percentage;
  if (nh.getParam("/XViewBagReader/relabeling_percentage", relabeling_percentage)) {
    params->relabeling_percentage = relabeling_percentage;
  }

  std::string graph_construction_camera;
  if (nh.getParam("/XViewBagReader/graph_construction_camera",
                  graph_construction_camera)) {
    if(graph_construction_camera == "FRONT")
      params->graph_construction_camera = x_view_ros::CAMERA::FRONT;
    else if(graph_construction_camera == "BACK")
      params->graph_construction_camera = x_view_ros::CAMERA::BACK;
    else if(graph_construction_camera == "RIGHT")
      params->graph_construction_camera = x_view_ros::CAMERA::RIGHT;
    else if(graph_construction_camera == "DOWN")
      params->graph_construction_camera = x_view_ros::CAMERA::DOWN;
    else
      LOG(ERROR) << "Unrecognized graph construction camera type <"
                 << graph_construction_camera << ".";
  }

  std::string localization_camera;
  if (nh.getParam("/XViewBagReader/localization_camera",
                  localization_camera)) {
    if(localization_camera == "FRONT")
      params->localization_camera = x_view_ros::CAMERA::FRONT;
    else if(localization_camera == "BACK")
      params->localization_camera = x_view_ros::CAMERA::BACK;
    else if(localization_camera == "RIGHT")
      params->localization_camera = x_view_ros::CAMERA::RIGHT;
    else if(localization_camera == "DOWN")
      params->localization_camera = x_view_ros::CAMERA::DOWN;
    else
      LOG(ERROR) << "Unrecognized localization camera type <"
                 << graph_construction_camera << ".";
  }

  int local_graph_steps;
  if(nh.getParam("/XViewBagReader/local_graph_steps", local_graph_steps))
    params->local_graph_steps = local_graph_steps;

  std::string run_name;
  if(nh.getParam("/XViewBagReader/run_name", run_name))
    params->run_name = run_name;
}

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n============== Parsing Bag Node parameters ===============\n";

  BagNodeParameters bag_node_parameters;
  loadBagNodeParameters(node_handle, &bag_node_parameters);

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  // Ignore or activate the pause functions.
  x_view_ros::Pause::activate(bag_node_parameters.pause_active);

  // Set up the evaluation.
  x_view_evaluation::EvaluationParameters evaluation_parameters;
  evaluation_parameters.timer_type = bag_node_parameters.timer_type;

  x_view_evaluation::Evaluation evaluation(evaluation_parameters);

  const uint64_t start_frame = bag_node_parameters.start_frame;
  const uint64_t end_frame = bag_node_parameters.end_frame;

  // Build the semantic graph associated to the path specified in the
  // parameters passed to the iteration function.
  bag_reader.iterateBagFromTo(bag_node_parameters.graph_construction_camera,
                              start_frame, end_frame);

  // Store the evaluation to disk.
  const std::string evaluation_output_dir =
      x_view_evaluation::Path::generateDirectoryPath(node_handle,
                                                     bag_node_parameters.run_name);

  const std::string graph_building_suffix = "graph_building";
  if(!evaluation.time.writeToFolder(evaluation_output_dir, graph_building_suffix))
    LOG(ERROR) << "Impossible to log time evaluation to "
               << evaluation_output_dir << ".";

  // Extract the timer associated to the construction and store it locally.
  x_view::AbstractTimer* construction_timer;
  evaluation.time.storeTimer(&construction_timer);

  LOG(INFO) << "\n=============== Postprocessing XView Graph ===============\n";

  // Introduce noise to the global semantic graph by relabeling some vertices.
  const uint64_t vertex_relabeling_seed = 0;
  bag_reader.relabelGlobalGraphVertices(
      bag_node_parameters.relabeling_percentage, vertex_relabeling_seed);

  const x_view::Graph& database_graph = bag_reader.getGlobalGraph();

  LOG(INFO) << "\n======================= Localizing =======================\n";

  // Try to localize the following views inside the previously constructed
  // semantic graph.
  x_view_ros::Pause pause;
  const uint64_t local_graph_steps = bag_node_parameters.local_graph_steps;
  for(int i = start_frame; i + local_graph_steps < end_frame;) {
    if(!pause.isPaused()) {

      // Generate a query graph, and store the ground truth location of the
      // last point of view.
      x_view::LocalizationPair locations;
      x_view::Graph query_graph;
      std::vector<x_view::PoseId> pose_ids;

      if(!bag_reader.generateQueryGraph(
          bag_node_parameters.localization_camera, i, local_graph_steps,
          &query_graph, &pose_ids, &locations))
        LOG(ERROR) << "Could not generate query graph.";

      // Try to localize the query graph.
      x_view::GraphMatcher::IndexMatrixType candidate_matches;
      x_view::GraphMatcher::SimilarityMatrixType similarity_matrix;

      x_view::real_t error =
          bag_reader.localizeGraph(query_graph, pose_ids, &locations,
                                   &candidate_matches, &similarity_matrix);

      std::cout << candidate_matches << std::endl << std::endl;

      std::cout << "Localization " << i - start_frame + 1 << " of "
                << end_frame - local_graph_steps - start_frame << std::endl;
      LOG(INFO) << "Estimation: \n"
                << x_view::formatSE3(locations.estimated_pose, "\t") << "\n"
                << "True: \n"
                << x_view::formatSE3(locations.true_pose, "\t") << "\n"
                << "Error: " << error << std::endl;

      evaluation.localization.addLocalization(locations, error);
      evaluation.similarity.addSimilarities(database_graph, query_graph,
                                            similarity_matrix,
                                            candidate_matches);
      ++i;
    }
  }
  pause.terminate();

  const std::string localization_suffix = "localization";
  if(!evaluation.time.writeToFolder(evaluation_output_dir, localization_suffix))
    LOG(ERROR) << "Impossible to log time evaluation to "
               << evaluation_output_dir << ".";

  if(!evaluation.localization.writeToFolder(evaluation_output_dir, localization_suffix))
    LOG(ERROR) << "Impossible to log time evaluation to "
               << evaluation_output_dir << ".";

  const std::string similarity_suffix = "similarity";
  if(!evaluation.similarity.writeToFolder(evaluation_output_dir,
                                          similarity_suffix))
    LOG(ERROR) << "Impossible to log similarity evaluation to "
               << evaluation_output_dir << ".";


  x_view::finalizeLogging();

  return 0;
}

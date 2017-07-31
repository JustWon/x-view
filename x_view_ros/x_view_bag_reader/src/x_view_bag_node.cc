#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/x_view_tools.h>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  // Build the semantic graph associated to the path specified in the
  // parameters passed to the iteration function.
  bag_reader.iterateBagFromTo(x_view_ros::CAMERA::FRONT, 0, 200);

  // Try to localize the following views inside the previously constructed
  // semantic graph.
  x_view_ros::Pause pause;
  x_view::Statistics stats;
  const int steps = 4;
  for(int i = 0; i< 200;) {
    if(!pause.isPaused()) {
      std::pair<Eigen::Vector3d, Eigen::Vector3d> locations;
      bool localized =  bag_reader.localize_graph(x_view_ros::CAMERA::FRONT, i,
                                                  steps, &locations);
      if(localized) {
        std::cout << "Estimation: " << Eigen::RowVector3d(locations.first)
                  << "\n"
                  << "True: " << Eigen::RowVector3d(locations.second)
                  << std::endl;
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

  x_view::finalizeLogging();

  return 0;
}

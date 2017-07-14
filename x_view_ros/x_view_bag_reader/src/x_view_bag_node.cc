#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_bag_reader/x_view_pause.h>
#include <x_view_core/x_view_tools.h>
#include <thread>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  // Build the semantic graph associated to the path specified in the
  // parameters passed to the iteration function.
  bag_reader.iterateBagFromTo(x_view_ros::CAMERA::FRONT, 0, 40);

  // Try to localize the following views inside the previously constructed
  // semantic graph.
  x_view_ros::Pause pause;
  for(int i = 0; i< 40; ) {
    if(!pause.isPaused()) {
      bag_reader.localize(x_view_ros::CAMERA::BACK, i);
      ++i;
    }
  }
  pause.terminate();

  x_view::finalizeLogging();

  return 0;
}
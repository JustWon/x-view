#include <x_view_bag_reader/x_view_bag_reader.h>
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
  for(int i = 0; i< 40; ++i) {
    bag_reader.localize(x_view_ros::CAMERA::BACK, i);
    // Sleep for 500ms to give time to see the localization in RViz.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  x_view::finalizeLogging();

  return 0;
}
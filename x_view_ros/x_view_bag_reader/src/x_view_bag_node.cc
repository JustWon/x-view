#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_core/x_view_tools.h>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  // FIXME Something is wrong with frame 6 as it stops the workflow of graph
  // matcher.
  bag_reader.iterateBagFromTo("Stereo_Left/Omni_F/labels", 10, 30);
  bag_reader.iterateBagFromTo("Stereo_Left/Omni_F/labels", 10, 30);
  bag_reader.iterateBagFromTo("Stereo_Left/Omni_F/labels", 10, 30);

  x_view::finalizeLogging();

  return 0;
}
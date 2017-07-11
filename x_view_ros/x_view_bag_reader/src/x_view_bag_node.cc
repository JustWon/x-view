#include <x_view_bag_reader/x_view_bag_reader.h>
#include <x_view_core/x_view_tools.h>
#include <thread>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  LOG(INFO) << "\n=============== Running X-View Bag Reader ================\n";

  x_view_ros::XViewBagReader bag_reader(node_handle);

  bag_reader.iterateBagFromTo(x_view_ros::CAMERA::FRONT, 0, 40);

  for(int i = 0; i< 40; ++i) {
    bag_reader.localize(x_view_ros::CAMERA::FRONT, i);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  x_view::finalizeLogging();

  return 0;
}
#include <thread>

#include <ros/ros.h>

#include <x_view_bag_reader/x_view_bag_reader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  x_view_ros::XViewBagReader bag_reader(node_handle);

  bag_reader.iterateBagForwards("Stereo_Left/Omni_R/labels");

  return 0;
}



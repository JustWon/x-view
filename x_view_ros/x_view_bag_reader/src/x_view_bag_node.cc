#include <thread>

#include <ros/ros.h>

#include <x_view_bag_reader/x_view_bag_reader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  x_view_ros::XViewBagReader bag_reader(node_handle);


  bag_reader.iterateBagFromTo("Stereo_Left/Omni_F/labels", 0, 20);
  bag_reader.iterateBagFromTo("Stereo_Left/Omni_R/labels", 20, 0);

  return 0;
}



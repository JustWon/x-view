#include <thread>

#include <ros/ros.h>

#include <x_view_bag_reader/x_view_bag_reader.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "X_View_Bag_Reader");
  ros::NodeHandle node_handle("~");

  x_view_ros::XViewBagReader bag_reader(node_handle);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  return 0;
}



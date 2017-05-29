#include <x_view_node/x_view_worker.h>

#include <ros/ros.h>
#include <glog/logging.h>

#include <thread>

int main(int argc, char** argv) {

  ros::init(argc, argv, "X_View");
  ros::NodeHandle node_handle("~");

  x_view_ros::XViewWorker worker(node_handle);

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

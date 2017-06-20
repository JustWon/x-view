#include <x_view_node/x_view_worker.h>
#include <x_view_core/x_view_tools.h>

#include <ros/ros.h>
#include <glog/logging.h>

#include <thread>

int main(int argc, char** argv) {

  x_view::setupLogging(argv);

  ros::init(argc, argv, "X_View");
  ros::NodeHandle node_handle("~");
  
  LOG(INFO) << "\n=================== Running X-View Node ==================\n";

  x_view_ros::XViewWorker worker(node_handle);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    LOG(ERROR) << "Exception: " << e.what();
    return 1;
  }
  catch (...) {
    LOG(ERROR) << "Unknown Exception";
    return 1;
  }

  x_view::finalizeLogging();

  return 0;
}

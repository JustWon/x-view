#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

namespace x_view_ros {

class XViewWorker {

 public:
  explicit XViewWorker(ros::NodeHandle& n);
  ~XViewWorker();

 private:
  // Get ROS parameters.
  void getParameters();

  // TODO: Add further setters / getters where necessary.

  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber image_sub_;

  // Publishers.
  ros::Publisher matches_pub_;

  // Services.
  ros::ServiceServer trigger_xview_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

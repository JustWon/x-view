#ifndef X_VIEW_NODE_H_
#define X_VIEW_NODE_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>


// TODO(gawela): Move non-ros functions to non ros package.
namespace x_view_ros {

class XViewNode {

 public:
  explicit XViewNode(ros::NodeHandle& n);
  ~XViewNode();

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
#endif /* X_VIEW_NODE_H_ */

#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace x_view_ros {

class XViewWorker {

 public:
  explicit XViewWorker(ros::NodeHandle& n);
  ~XViewWorker();

 protected:

  /// \brief Process semantics image.
  void semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg);

 private:
  /// \brief Get ROS parameters.
  void getParameters();

  // TODO: Add further setters / getters where necessary.

  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber semantics_image_sub_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

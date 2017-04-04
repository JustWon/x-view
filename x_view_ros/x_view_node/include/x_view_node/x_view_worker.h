#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <x_view_core/x_view.h>

namespace x_view_ros {

class XViewWorker {

  struct XViewWorkerParams {
    std::string semantics_image_topic;
    x_view::XViewParams x_view_params;
  }; // struct XViewWorkerParams

 public:
  explicit XViewWorker(ros::NodeHandle& n);

  ~XViewWorker();

 protected:

  /// \brief Process semantics image generating internal representation of semantic entities.
  void semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg);

 private:
  /// \brief Get ROS parameters.
  void getParameters();

  // TODO: Add further setters / getters where necessary.

  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber semantics_image_sub_;

  // Parameters.
  XViewWorkerParams params_;

  // X_View core.
  x_view::XView x_view_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

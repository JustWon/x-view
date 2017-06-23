#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <x_view_core/x_view.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_parser/parser.h>

//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <kindr/minimal/quat-transformation.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

namespace x_view_ros {

class XViewWorker {

 public:
  explicit XViewWorker(ros::NodeHandle& n);

  ~XViewWorker();

 protected:

  /// \brief Process semantics image generating internal representation of semantic entities.
  void semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg);

 private:

  void tfTransformToSE3(const tf::StampedTransform& tf_transform,
                        x_view::SE3* pose);

  void parseParameters() const;

  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber semantics_image_sub_;

  // Transform communication.
  tf::TransformListener tf_listener_;

  // X_View core.
  std::unique_ptr<x_view::XView> x_view_;

  // Parser object
  Parser parser_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

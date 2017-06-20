#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <x_view_core/x_view.h>
#include <x_view_core/datasets/abstract_dataset.h>

//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <kindr/minimal/quat-transformation.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

namespace x_view_ros {

class XViewWorker {

  struct XViewWorkerParams {
    std::string dataset_name;
    std::string semantics_image_topic;

    std::string world_frame;
    std::string sensor_frame;
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

  void tfTransformToSE3(const tf::StampedTransform& tf_transform,
                        x_view::SE3* pose);

  // Node handle.
  ros::NodeHandle& nh_;

  // Subscribers.
  ros::Subscriber semantics_image_sub_;

  // Transform communication.
  tf::TransformListener tf_listener_;

  // Parameters.
  XViewWorkerParams params_;

  // X_View core.
  x_view::XView x_view_;

  // Dataset handler
  std::shared_ptr<x_view::AbstractDataset> dataset_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

#include <x_view_node/x_view_worker.h>
#include <x_view_core/datasets/synthia_dataset.h>

#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

namespace enc = sensor_msgs::image_encodings;

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n) {
  getParameters();
  semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                       &XViewWorker::semanticsImageCallback,
                                       this);

  x_view_ = x_view::XView(params_.x_view_params);

  // set the usage of the specified dataset
  if (params_.dataset_name.compare("SYNTHIA") == 0)
    dataset_.reset(new x_view::SynthiaDataset);
  else
    CHECK(false) << "Dataset '" << params_.dataset_name
    << "' is not supported" << std::endl;
}

XViewWorker::~XViewWorker() {
}

void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  // Preprocess the ros message.
  cv::Mat image = dataset_->convertSemanticImage(msg);

  // Read in pose in world frame.
  tf::StampedTransform tf_transform;
  if (tf_listener_.waitForTransform(params_.world_frame, params_.sensor_frame,
                                    msg->header.stamp,
                                    ros::Duration(0.2))) {
    // Get the tf transform.
    tf_listener_.lookupTransform(params_.world_frame, params_.sensor_frame,
                                 msg->header.stamp, tf_transform);
  } else {
    ROS_ERROR_STREAM(
        "Failed to get transformation between " << params_.world_frame
        << " and " << params_.sensor_frame);
  }
  x_view::SE3 pose;
  tfTransformToSE3(tf_transform, &pose);
  x_view_.processSemanticImage(image, pose);
}

void XViewWorker::getParameters() {

  // XView parameters.
  if (!nh_.getParam("/XView/landmarks/type", params_
                    .x_view_params.semantic_landmark_type_)) {
    ROS_ERROR_STREAM("Failed to get param '/XView/landmarks/type'\nUsing "
        "default <SURF> landmark type.");
    params_.x_view_params.semantic_landmark_type_ = "SURF";
  }

  if (!nh_.getParam("/XView/matcher/type", params_
                    .x_view_params.landmark_matching_type_)) {
    ROS_ERROR_STREAM("Failed to get param '/XView/matcher/type'\nUsing "
        "default <VISUAL> landmark matcher.");
    params_.x_view_params.landmark_matching_type_ = "VECTOR";
  }


  // XViewWorker parameters.
  if (!nh_.getParam("/XViewWorker/dataset", params_.dataset_name)) {
    ROS_ERROR_STREAM("Failed to get param '/XViewWorker/dataset'\nUsing "
        "default <SYNTHIA> dataset.");
    params_.dataset_name = "SYNTHIA";
  }
  params_.x_view_params.semantic_dataset_name_ = params_.dataset_name;

  if (!nh_.getParam("/XViewWorker/semantics_image_topic",
                    params_.semantics_image_topic)) {
    ROS_ERROR_STREAM("Failed to get param "
        "'/XViewWorker/semantics_image_topic'");
  }
  if (!nh_.getParam("/XViewWorker/world_frame",
                    params_.world_frame)) {
    ROS_ERROR_STREAM("Failed to get param "
        "'/XViewWorker/world_frame'");
  }
  if (!nh_.getParam("/XViewWorker/sensor_frame",
                    params_.sensor_frame)) {
    ROS_ERROR_STREAM("Failed to get param "
        "'/XViewWorker/sensor_frame'");
  }
}

void XViewWorker::tfTransformToSE3(const tf::StampedTransform& tf_transform,
                                   x_view::SE3* pose) {
  // Register new pose.
  x_view::SE3::Position pos(
      tf_transform.getOrigin().getX(),
      tf_transform.getOrigin().getY(),
      tf_transform.getOrigin().getZ());
  x_view::SE3::Rotation::Implementation rot(
      tf_transform.getRotation().getW(),
      tf_transform.getRotation().getX(),
      tf_transform.getRotation().getY(),
      tf_transform.getRotation().getZ());
  *pose = x_view::SE3(pos, rot);
}
}

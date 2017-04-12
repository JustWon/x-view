#include <x_view_node/x_view_worker.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n) {
  getParameters();
  semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                       &XViewWorker::semanticsImageCallback,
                                       this);

  x_view_ = x_view::XView(params_.x_view_params);
}

XViewWorker::~XViewWorker() {
}

void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;


    x_view_.process(image, x_view::SE3());
    // TODO: Process image using x-view functions.

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM(
        "Could not convert from " << msg->encoding.c_str() << " to 'bgr8'.");
  }
}

void XViewWorker::getParameters() {

  // XView parameters.
  if (nh_.getParam("/XView/semantics/dataset", params_
      .x_view_params.semantic_dataset_name_)) {
    ROS_INFO_STREAM(
        "XView is working on the following semantic dataset: <"
            << params_.x_view_params.semantic_dataset_name_ << ">");
  } else {
    ROS_FATAL_STREAM("Failed to get param '/XView/semantics/dataset'");
  }

  if (nh_.getParam("/XView/landmarks/type", params_
      .x_view_params.semantic_landmark_type_)) {
    ROS_INFO_STREAM(
        "XView is using the following semantic landmark type: <"
            << params_.x_view_params.semantic_landmark_type_ << ">");
  } else {
    ROS_ERROR_STREAM("Failed to get param '/XView/landmarks/type'\nUsing "
                         "default <SURF> landmark type.");
    params_.x_view_params.semantic_landmark_type_ = "SURF";
  }

  if (nh_.getParam("/XView/matcher/type", params_
      .x_view_params.landmark_matching_type_)) {
    ROS_INFO_STREAM(
        "XView is using the following landmark matcher type: <"
            << params_.x_view_params.landmark_matching_type_ << ">");
  } else {
    ROS_ERROR_STREAM("Failed to get param '/XView/matcher/type'\nUsing "
                         "default <VISUAL> landmark matcher.");
    params_.x_view_params.landmark_matching_type_ = "VECTOR";
  }


  // XViewWorker parameters.
  nh_.getParam("/XViewWorker/semantics_image_topic",
               params_.semantics_image_topic);
}

}

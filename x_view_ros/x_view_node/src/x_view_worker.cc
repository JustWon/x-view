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

  // preprocess the ros message
  cv::Mat image = dataset_->convertSemanticImage(msg);

  x_view_.processSemanticImage(image, x_view::SE3());

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
}

}

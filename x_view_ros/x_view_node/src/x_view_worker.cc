#include <x_view_node/x_view_worker.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = image_ptr->image;

    // add the gathered image to the x-view
    // TODO: Process image using x-view functions.

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM(
        "Could not convert from " << msg->encoding.c_str() << " to 'bgr8'.");
  }
}

void XViewWorker::getParameters() {

  std::string semanticLandmarkTypeString;
  if (nh_.getParam("/XView/landmarks/type", semanticLandmarkTypeString)) {
    ROS_INFO_STREAM(
        "XView is using the following semantic landmark type: <"
            << semanticLandmarkTypeString << ">");
    if (semanticLandmarkTypeString.compare("bos_histogram") == 0) {
      params_.x_view_params.semantic_landmark_type_ =
          x_view::SemanticLandmarkFactory::SEMANTIC_LANDMARK_TYPE::BOS_HISTOGRAM;
    } else if (semanticLandmarkTypeString.compare("bos_visual") == 0) {
      params_.x_view_params.semantic_landmark_type_ =
          x_view::SemanticLandmarkFactory::SEMANTIC_LANDMARK_TYPE
          ::BOS_VISUAL_FEATURE;
    } else if (semanticLandmarkTypeString.compare("graph") == 0) {
      params_.x_view_params.semantic_landmark_type_ =
          x_view::SemanticLandmarkFactory::SEMANTIC_LANDMARK_TYPE::GRAPH;
    } else {
      ROS_ERROR_STREAM(
          "Parameter associated to 'semanticLandmarkType' is unknown:\n\tgiven: "
              << semanticLandmarkTypeString << "\n\tvalid: {'bos', 'graph'}");
    }

  } else {
    ROS_ERROR_STREAM("Failed to get param 'semanticLandmarkType'");
  }

  // XView parameters.
  nh_.getParam("/XView/placeholder", params_.x_view_params.placeholder);

  // XViewWorker parameters.
  nh_.getParam("/XViewWorker/semantics_image_topic",
               params_.semantics_image_topic);
}

}

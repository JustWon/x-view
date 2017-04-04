#include "../include/x_view_node/x_view_worker.h"

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

XViewWorker::~XViewWorker()
{
}

void XViewWorker::semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::Mat image = image_ptr->image;
    // TODO: Process image using x_view functions.
    CHECK(false) << "Not implemented.";
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void XViewWorker::getParameters()
{
  // XView parameters.
  nh_.getParam("/XView/placeholder",
                 params_.x_view_params.placeholder);

  // XViewWorker parameters.
  nh_.getParam("/XViewWorker/semantics_image_topic",
               params_.semantics_image_topic);
}

}

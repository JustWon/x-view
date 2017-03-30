#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

#include "x-view_node/x-view_worker.h"

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n) {
  semantics_image_sub_ = nh_.subscribe(params_.semantics_image_topic, 1,
                                       &XViewWorker::semanticsImageCallback,
                                       this);
  getParameters();
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
    // TODO: Process image using x-view functions.
    CHECK(false) << "Not implemented.";
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void XViewWorker::getParameters()
{
  // TODO: read in parameters from node.
  CHECK(false) << "Not implemented.";
}

}

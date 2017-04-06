#include <x_view_node/x_view_parameter_handler.h>

namespace x_view_ros {

std::unique_ptr<XViewParameterHandler>
    XViewParameterHandler::parameter_handler_;

void XViewParameterHandler::initialize(const ros::NodeHandle& nh) {
  if (parameter_handler_)
    ROS_ERROR_STREAM("Parameter handler has already been initialized");
  else
    parameter_handler_.reset(new XViewParameterHandler(nh));
}

const std::unique_ptr<XViewParameterHandler>& XViewParameterHandler::instance() {
  if (!parameter_handler_)
    ROS_ERROR_STREAM("Parameter handle has not been initialized");
  return parameter_handler_;
}

XViewParameterHandler::XViewParameterHandler(const ros::NodeHandle& nh):nh_(nh)
{
  ROS_INFO_STREAM("Initializing the parameter  handler");
}

}


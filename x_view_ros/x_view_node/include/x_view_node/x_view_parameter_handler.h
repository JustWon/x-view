#ifndef X_VIEW_X_VIEW_PARAMETER_HANDLER_H
#define X_VIEW_X_VIEW_PARAMETER_HANDLER_H

#include <ros/ros.h>

#include <memory>

namespace x_view_ros {

class XViewParameterHandler {

 public:
  static void initialize(const ros::NodeHandle& nh);

  static const std::unique_ptr<XViewParameterHandler>& instance();

 private:
  XViewParameterHandler(const ros::NodeHandle& nh);

 private:
  static std::unique_ptr<XViewParameterHandler> parameter_handler_;
  const ros::NodeHandle& nh_;
};

}

#endif //X_VIEW_X_VIEW_PARAMETER_HANDLER_H

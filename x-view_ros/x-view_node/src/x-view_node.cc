#include "x-view_node/x-view_node.h"

namespace x_view_ros {

XViewNode::XViewNode(ros::NodeHandle& n) : nh_(n) { }

void XViewNode::getParameters() {
  // TODO: read in parameters from node.
  CHECK(false) << "Not implemented.";
}

}

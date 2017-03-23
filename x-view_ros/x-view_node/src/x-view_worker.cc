#include "x-view_node/x-view_worker.h"

namespace x_view_ros {

XViewWorker::XViewWorker(ros::NodeHandle& n) : nh_(n) {}

XViewWorker::~XViewWorker() {}

void XViewWorker::getParameters() {
  // TODO: read in parameters from node.
  CHECK(false) << "Not implemented.";
}

}

#include "x-view_node/x-view_node.h"

namespace x_view_ros {

XViewNode::XViewNode(ros::NodeHandle& n) : nh_(n) { }

void XViewNode::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                          XViewSemantics* semantics_out) {
  // TODO: build semantic descriptor from image input.
  CHECK(false) << "Not implemented.";
}

void XViewNode::matchSemantics(const XViewSemantics& semantics_a,
                               Eigen::MatrixXd* matches) {
  // TODO: match input semantics against semantics_db_.
  CHECK(false) << "Not implemented.";
}

void XViewNode::filterMatches(const XViewSemantics& semantics_a,
                              Eigen::MatrixXd* matches) {
  // TODO: filter matches, e.g., with geometric verification.
  CHECK(false) << "Not implemented.";
}

void XViewNode::mergeSemantics(const XViewSemantics& semantics_a,
                               const Eigen::MatrixXd& matches) {
  // TODO: Merge semantics with semantics_db_ if dominant matches,
  // otherwise add semantics as new instance to semantics_db_.
  // TODO: use filterMatches function before merging.
  CHECK(false) << "Not implemented.";
}

void XViewNode::cleanDatabase() {
  // TODO: sweep over semantics_db_ to match and merge unconnected semantics.
  CHECK(false) << "Not implemented.";
}

void XViewNode::getParameters() {
  // TODO: read in parameters from node.
  CHECK(false) << "Not implemented.";
}

}

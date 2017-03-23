#ifndef X_VIEW_NODE_H_
#define X_VIEW_NODE_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>


// TODO(gawela): Move non-ros functions to non ros package.
namespace x_view_ros {
typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

struct XViewParams { }; // struct XViewParams

struct XViewSemantics {
  SE3 pose;
  gtsam::NonlinearFactorGraph& factor_graph;
}; // struct XViewSemantics

class XViewNode {

 public:
  explicit XViewNode(ros::NodeHandle& n);
  ~XViewNode();

  /// \brief Extract semantic descriptor from semantics image.
  void extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                 XViewSemantics* semantics_out);

  /// \brief Match semantics instance to database and return score.
  void matchSemantics(const XViewSemantics& semantics_a,
                      Eigen::MatrixXd* matches);

  /// \brief Filter matches, e.g., geometric verification etc.
  void filterMatches(const XViewSemantics& semantics_a,
                     Eigen::MatrixXd* matches);

  /// \brief Merge semantics instance into database according to matches.
  void mergeSemantics(const XViewSemantics& semantics_a,
                      const Eigen::MatrixXd& matches);

  /// \brief Clean database by doing full semantics matching.
  void cleanDatabase();

 private:
  // Get ROS parameters.
  void getParameters();

  // TODO: Add further setters / getters where necessary.

  // Node handle.
  ros::NodeHandle& nh_;

  // Parameters.
  XViewParams params_;

  // Subscribers.
  ros::Subscriber image_sub_;

  // Publishers.
  ros::Publisher matches_pub_;

  // Services.
  ros::ServiceServer trigger_xview_;

  std::vector<XViewSemantics> semantics_db_;
}; // XViewNode

}
#endif /* X_VIEW_NODE_ */

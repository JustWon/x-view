#include <x_view_core/x_view.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/highgui/highgui.hpp>

namespace x_view {

XView::XView(XViewParams& params) : params_(params) {

  // create a factory object which is responsible for generating new semantic landmark observations
  semantic_factory_.setSemanticLandmarkType(params_.semantic_landmark_type_);
}

XView::~XView() {};

void XView::process(const cv::Mat& image, const SE3& pose) {
  // generate a new semantic landmark object
  SemanticLandmarkPtr landmarkPtr;

  cv::namedWindow("Parsed image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Parsed image", image);
  cv::waitKey(1000);
  cv::destroyWindow("Parsed image");

  // extract associated semantics
  extractSemanticsFromImage(image, pose, landmarkPtr);

  // TODO: call other functions like "matchSemantics" etc. here
}

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // create the actual landmark representation
  semantics_out = semantic_factory_.createSemanticLandmark(image, pose);

  // TODO: add the semantics_out landmark to the database
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           Eigen::MatrixXd& matches) {
  // TODO: match input semantics against semantics_db_ by doing a sort of loop and calling "this->semantics_db_[i].match(semantics_a)".
  CHECK(false) << "Not implemented.";
}

void XView::filterMatches(const SemanticLandmarkPtr& semantics_a,
                          Eigen::MatrixXd& matches) {
  // TODO: filter matches, e.g., with geometric verification.
  CHECK(false) << "Not implemented.";
}

void XView::mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                           const Eigen::MatrixXd& matches) {
  // TODO: Merge semantics with semantics_db_ if dominant matches,
  // otherwise add semantics as new instance to semantics_db_.
  // TODO: use filterMatches function before merging.
  CHECK(false) << "Not implemented.";
}

void XView::cleanDatabase() {
  // TODO: sweep over semantics_db_ to match and merge unconnected semantics.
  CHECK(false) << "Not implemented.";
}

}

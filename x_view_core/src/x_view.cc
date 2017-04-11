#include <x_view_core/x_view.h>
#include <x_view_core/visual_feature.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace x_view {

XView::XView(XViewParams& params) : params_(params) {
  // create a factory object which is responsible for generating new semantic landmark observations
  if (params.semantic_landmark_type_string_.compare("ORB") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::ORB_VISUAL_FEATURE;
    semantic_factory_.setCreatorFunction(ORBVisualFeature::create);
  } else if (params.semantic_landmark_type_string_.compare("SIFT") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SIFT_VISUAL_FEATURE;
    semantic_factory_.setCreatorFunction(SIFTVisualFeature::create);
  } else if (params.semantic_landmark_type_string_.compare("SURF") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SURF_VISUAL_FEATURE;
    semantic_factory_.setCreatorFunction(SURFVisualFeature::create);
  }
}

XView::~XView() {};

void XView::process(const cv::Mat& image, const SE3& pose) {
  // generate a new semantic landmark object
  SemanticLandmarkPtr landmarkPtr;

  // extract associated semantics
  extractSemanticsFromImage(image, pose, landmarkPtr);

  // display image with detected keypoints
  cv::Mat imageWithFeatures;
  cv::drawKeypoints(image, std::dynamic_pointer_cast<VisualFeature>
      (landmarkPtr)->keypoints_, imageWithFeatures);
  cv::imshow("Extracted visual features", imageWithFeatures);
  cv::waitKey(500);

  LOG(INFO) << "Detected " << std::dynamic_pointer_cast<VisualFeature>
      (landmarkPtr)->keypoints_.size() << " keypoints" << std::endl;

  // TODO: call other functions like "matchSemantics" etc. here
}

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // create the actual landmark representation
  semantics_out = semantic_factory_.createSemanticLandmark(image, pose);
  semantics_db_.push_back(semantics_out);
  std::cout << "Semantic database has size: "
            << semantics_db_.size() << std::endl;

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

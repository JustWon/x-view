#include <x_view_core/x_view.h>
#include <x_view_core/landmarks/visual_feature.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace x_view {

XView::XView(XViewParams& params) : params_(params) {
  // create a factory object which is responsible for generating new semantic landmark observations
  if (params.semantic_landmark_type_string_.compare("ORB") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::ORB_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(ORBVisualFeature::create);
  } else if (params.semantic_landmark_type_string_.compare("SIFT") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SIFT_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(SIFTVisualFeature::create);
  } else if (params.semantic_landmark_type_string_.compare("SURF") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SURF_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(SURFVisualFeature::create);
  }
}

XView::~XView() {};

void XView::process(const cv::Mat& image, const SE3& pose) {
  // generate a new semantic landmark object
  SemanticLandmarkPtr landmarkPtr;

  // extract associated semantics
  extractSemanticsFromImage(image, pose, landmarkPtr);

  // display image with detected keypoints
//  cv::Mat imageWithFeatures;
//  cv::drawKeypoints(image, std::dynamic_pointer_cast<VisualFeature>
//      (landmarkPtr)->keypoints_, imageWithFeatures);
//  cv::imshow("Extracted visual features", imageWithFeatures);
//  cv::waitKey(500);
//
//  LOG(INFO) << "Detected " << std::dynamic_pointer_cast<VisualFeature>
//      (landmarkPtr)->keypoints_.size() << " keypoints" << std::endl;

  // compute the matchings between the new feature and the ones present in
  // the database
  Eigen::MatrixXd matchingScores;
  matchSemantics(landmarkPtr, matchingScores);

  // TODO: call other functions like "matchSemantics" etc. here
}

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // create the actual landmark representation
  semantics_out =
      semantic_landmark_factory_.createSemanticLandmark(image, pose);

  // TODO: post process the semantic landmark representation given its
  // neighbors stored in the semantic database
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           Eigen::MatrixXd& matches) {
  std::shared_ptr<const VisualFeature> sem_a =
      std::dynamic_pointer_cast<const VisualFeature>(semantics_a);

  // iterate over the existing semantic landmarks and store their similarity
  for (int l = 0; l < semantics_db_.size(); ++l) {
    const SemanticLandmarkPtr& semantics_b = semantics_db_[l];

    std::shared_ptr<const VisualFeature> sem_b =
        std::dynamic_pointer_cast<const VisualFeature>(semantics_b);

    SemanticMatchingResult comparison_result = sem_a->match(sem_b);

    std::cerr<<"Computing and showing matches between current frame (left) "
        "and " << l+1<< "th frame stored in the database" << std::endl;

    if (comparison_result.matches_.size() > 0) {
      cv::Mat img_matches;
      cv::drawMatches(semantics_a->image_, sem_a->keypoints_,
                      semantics_b->image_, sem_b->keypoints_,
                      comparison_result.matches_,
                      img_matches);

      cv::imshow("Matches", img_matches);
      cv::waitKey(2000);
    }

  }

  semantics_db_.push_back(semantics_a);
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

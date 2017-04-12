#include <x_view_core/x_view.h>
#include <x_view_core/landmarks/visual_feature.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CAST(object, type) std::dynamic_pointer_cast<type>(object)

namespace x_view {

XView::XView(XViewParams& params) : params_(params) {
  // create a factory object which is responsible for generating new semantic landmark observations
  if (params_.semantic_landmark_type_.compare("ORB") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::ORB_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(ORBVisualFeature::create);
  } else if (params_.semantic_landmark_type_.compare("SIFT") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SIFT_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(SIFTVisualFeature::create);
  } else if (params_.semantic_landmark_type_.compare("SURF") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SURF_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction(SURFVisualFeature::create);
  }

  // set a landmark matcher
  if (params_.landmark_matching_type_.compare("VISUAL") == 0) {
    landmarks_matcher_type_ = LandmarksMatcherType::VISUAL_FEATURES_MATCHER;
    descriptor_matcher_ = VisualFeaturesMatcher::create();
  }
}

XView::~XView() {};

void XView::process(const cv::Mat& image, const SE3& pose) {
  // generate a new semantic landmark object
  SemanticLandmarkPtr landmarkPtr;

  // extract associated semantics
  extractSemanticsFromImage(image, pose, landmarkPtr);

  // add the features to the matcher
  std::shared_ptr<VisualFeature> vPtr = CAST(landmarkPtr, VisualFeature);

  // compute the matches between the new feature and the ones
  // stored in the database
  Eigen::MatrixXd matchingScores;
  matchSemantics(landmarkPtr, matchingScores);

  // add the newly computed descriptor to the descriptor matcher
  CAST(descriptor_matcher_, VectorFeaturesMatcher)->add_descriptor(vPtr->descriptors_);

  // TODO: call other functions to process semantic landmarks here
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
                           Eigen::MatrixXd& matches_mat) {

  std::shared_ptr<const VisualFeature> sem_a =
      CAST(semantics_a, const VisualFeature);

  std::vector<std::vector<cv::DMatch>> matches;
  CAST(descriptor_matcher_, VectorFeaturesMatcher)->match(sem_a->descriptors_, matches);

  const unsigned long number_of_training_images = semantics_db_.size();

  if (number_of_training_images > 0) {
    std::vector<int> voting_per_image(number_of_training_images, 0);
    for (int feature = 0; feature < matches.size(); ++feature) {
      for (int vote = 0; vote < matches[feature].size(); ++vote) {
        int preference = matches[feature][vote].imgIdx;
        voting_per_image[preference]++;
      }
    }

    auto max_vote = std::max_element(voting_per_image.begin(), voting_per_image
        .end());

    std::cout << "Current frame (" << number_of_training_images
              << ") voted " << 100 * ((double) *max_vote) / (matches.size())
              << "% for image "
              << std::distance(voting_per_image.begin(), max_vote)
              << std::endl;

    std::cout << std::endl;
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

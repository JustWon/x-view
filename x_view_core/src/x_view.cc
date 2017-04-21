#include <x_view_core/x_view.h>
#include <x_view_core/features/visual_feature.h>
#include <x_view_core/landmarks/visual_feature_landmark.h>
#include <x_view_core/matchers/vector_matcher.h>
#include <x_view_core/datasets/synthia_dataset.h>
#include <x_view_core/datasets/airsim_dataset.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <x_view_core/landmarks/histogram_landmark.h>

namespace x_view {

XView::XView(XViewParams& params) : params_(params) {
  // parse the passed parameters and instantiate concrete classes for all
  // class members
  parseParameters();

  // print XView parameters and infos
  printInfo();
}

XView::~XView() {};

void XView::processSemanticImage(const cv::Mat& image, const SE3& pose) {

  // generate a new semantic landmark pointer
  SemanticLandmarkPtr landmarkPtr;

  // extract semantics associated to the image and pose
  extractSemanticsFromImage(image, pose, landmarkPtr);

  // compute the matches between the new feature and the ones
  // stored in the database
  AbstractMatcher::MatchingResultPtr matchingResult;
  matchSemantics(landmarkPtr, matchingResult);

  // add the newly computed descriptor to the descriptor matcher
  descriptor_matcher_->addLandmark(landmarkPtr);

  // TODO: call other functions to process semantic landmarks here
}

void XView::printInfo() const {
  std::string info = "\n";
  info += "==============================================================";
  info += "\n                          XView";
  info += "\n" + dataset_->datasetInfo("\t");
  info += "\tLandmark type:\t<" + params_.semantic_landmark_type_ + ">";
  info += "\n\tMatcher type: \t<" + params_.landmark_matching_type_ + ">";
  info += "\n===============================================================";

  std::cout << info << std::endl;
}

void XView::parseParameters() {
  // define the dataset being used
  parseDatasetType();

  // define landmark type
  parseLandmarkType();

  // defiene a landmark matcher
  parseMatcherType();

}

void XView::parseDatasetType() {

  if (params_.semantic_dataset_name_.compare("SYNTHIA") == 0) {
    dataset_ = ConstDatasetPrt(new SynthiaDataset);
  } else if (params_.semantic_dataset_name_.compare("AIRSIM") == 0) {
    dataset_ = ConstDatasetPrt(new AirsimDataset);
  } else {
    CHECK(false) << "Unrecognized dataset type <" << params_
        .semantic_dataset_name_ << ">" << std::endl;
  }
}

void XView::parseLandmarkType() {

  if (params_.semantic_landmark_type_.compare("ORB") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::ORB_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (ORBVisualFeatureLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("SIFT") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SIFT_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (SIFTVisualFeatureLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("SURF") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SURF_VISUAL_FEATURE;
    semantic_landmark_factory_.setCreatorFunction
        (SURFVisualFeatureLandmark::create);
  } else if (params_.semantic_landmark_type_.compare("HISTOGRAM") == 0) {
    semantic_landmark_type_ = SemanticLandmarkType::SEMANTIC_HISTOGRAM;
    semantic_landmark_factory_.setCreatorFunction
        (HistogramLandmark::create);
  } else {
    CHECK(false) << "Unrecognized landmark type <" << params_
        .semantic_landmark_type_ << ">" << std::endl;
  }

}

void XView::parseMatcherType() {
  if (params_.landmark_matching_type_.compare("VECTOR") == 0) {
    landmarks_matcher_type_ = LandmarksMatcherType::VECTOR_MATCHER;
    descriptor_matcher_ = VectorMatcher::create();
  } else {
    CHECK(false) << "Unrecognized matcher type <" << params_
        .landmark_matching_type_ << ">" << std::endl;
  }
}

//==========================================================================//
//         FUNCTIONS CALLED BY 'processSemanticImage' FUNCTION              //
//==========================================================================//

void XView::extractSemanticsFromImage(const cv::Mat& image, const SE3& pose,
                                      SemanticLandmarkPtr& semantics_out) {

  // TODO: preprocess image and pose

  // create the actual landmark representation whose implementation depends
  // on the parameters passed to XView
  semantics_out =
      semantic_landmark_factory_.createSemanticLandmark(image, pose);

  // TODO: post process the semantic landmark representation given its neighbors stored in the semantic database
}

void XView::matchSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matchingResult) {

  // compute a match between the current semantic landmark and the ones
  // already visited
  descriptor_matcher_->match(semantics_a, matchingResult);

  // The code inside these brackets is here only for log purposes
  {

    // FIXME: depending on the feature types, we extract the matches in a different way, how to encapsulate this in the Abstract interface?
    auto matching =
        std::dynamic_pointer_cast<VectorMatcher::VectorMatchingResult>
            (matchingResult);

    std::vector<std::vector<cv::DMatch>> matches = matching->matches;

    const unsigned long number_of_training_images = semantics_db_.size();

    if (number_of_training_images > 0) {
      std::vector<int> voting_per_image(number_of_training_images, 0);
      for (int feature = 0; feature < matches.size(); ++feature) {
        for (int vote = 0; vote < matches[feature].size(); ++vote) {
          int preference = matches[feature][vote].imgIdx;
          voting_per_image[preference]++;
        }
      }

      auto
          max_vote = std::max_element(voting_per_image.begin(), voting_per_image
          .end());

      std::cout << "Current frame (" << number_of_training_images
                << ") voted " << 100 * ((double) *max_vote) / (matches.size())
                << "% for image "
                << std::distance(voting_per_image.begin(), max_vote)
                << std::endl;
    }
  }

  semantics_db_.push_back(semantics_a);
}

void XView::filterMatches(const SemanticLandmarkPtr& semantics_a,
                          AbstractMatcher::MatchingResultPtr& matchingResult) {
  // TODO: filter matches, e.g., with geometric verification.
  CHECK(false) << "Not implemented.";
}

void XView::mergeSemantics(const SemanticLandmarkPtr& semantics_a,
                           AbstractMatcher::MatchingResultPtr& matchingResult) {
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

#include <x_view_core/landmarks/visual_feature.h>
#include <opencv2/features2d/features2d.hpp>

namespace x_view {

// TODO: read number of desired features from config file
int VisualFeature::NUM_VISUAL_FEATURES = 1000;

VisualFeature::VisualFeature(const cv::Mat& image, const SE3& pose)
    : AbstractSemanticLandmark(image, pose) {
}

SemanticMatchingResult
VisualFeature::match(const ConstSemanticLandmarkPtr& other) const {

  std::shared_ptr<const VisualFeature> semantics_b =
      std::dynamic_pointer_cast<const VisualFeature>(other);

  cv::BFMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(this->descriptors_, semantics_b->descriptors_, matches);

  double max_dist = 0;
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < this->descriptors_.rows; i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  // Draw only "good" matches (i.e. whose distance is less than 5*min_dist,
  // or a small arbitary value ( 0.02 ) in the event that min_dist is very small
  std::vector<cv::DMatch> good_matches;

  for (int i = 0; i < this->descriptors_.rows; i++) {
    if (matches[i].distance <= std::max(5 * min_dist, 0.05))
      good_matches.push_back(matches[i]);
  }

  SemanticMatchingResult result;
  result.score_ = 1.0;
  result.matches_ = std::move(good_matches);

  return result;
}

}

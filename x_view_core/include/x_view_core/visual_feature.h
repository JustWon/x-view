#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/features2d/features2d.hpp>

namespace x_view {

struct VisualFeature : public AbstractSemanticLandmark {

  explicit VisualFeature(const cv::Mat& image, const SE3& pose,
                         int num_desired_visual_features);

  virtual SemanticMatchingResult
  match(const AbstractSemanticLandmark& other)  = 0;

  const int num_desired_visual_features_;

  std::unique_ptr<cv::Feature2D> features_extractor_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

};
}

#endif // X_VIEW_BOS_H_

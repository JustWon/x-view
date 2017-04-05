#ifndef X_VIEW_BOS_H_
#define  X_VIEW_BOS_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/abstract_semantic_landmark.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace x_view {

struct BoS : public AbstractSemanticLandmark {

  explicit BoS(const cv::Mat& image, const SE3& pose);

  virtual SemanticMatchingResult
  match(const AbstractSemanticLandmark& other)  = 0;

};

struct BoSHistogram : public BoS {

  explicit BoSHistogram(const cv::Mat& image, const SE3& pose)
      : BoS(image, pose) {};

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other) {};

  std::vector<int> semantic_entity_count_;
};

template<typename T>
struct BoSVisualFeatures : public BoS {

  typedef T VisualFeatureType;

  explicit BoSVisualFeatures(const cv::Mat& image, const SE3& pose,
                             const int num_desired_visual_features)
      : BoS(image, pose),
        num_desired_visual_features_(num_desired_visual_features) {
    feature_.reset(new cv::ORB(num_desired_visual_features_));
    feature_->compute(image, keypoints_, descriptors_);
  }

  virtual SemanticMatchingResult match(const AbstractSemanticLandmark& other);

  const int num_desired_visual_features_;

  std::shared_ptr<VisualFeatureType> feature_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;
};

}

#endif // X_VIEW_BOS_H_

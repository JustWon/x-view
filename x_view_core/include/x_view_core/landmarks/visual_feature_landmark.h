#ifndef X_VIEW_VISUAL_FEATURE_LANDMARK_H_
#define  X_VIEW_VISUAL_FEATURE_LANDMARK_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace x_view {

/**
 * \brief A VisualFeatureLandmark is a landmark containing features
 * typically extracted in classic computer vision algorithms such as SIFT, ORB etc.
 * This is an abstract class (interface) implemented by all landmarks based
 * on classical visual features
 */
class VisualFeatureLandmark : public AbstractSemanticLandmark {

 public:
  explicit VisualFeatureLandmark(const cv::Mat& image, const SE3& pose);

  ///\brief number of features to be extracted by the detector
  static int NUM_VISUAL_FEATURES;

  std::unique_ptr<cv::Feature2D> features_extractor_;
  ///\brief set of keypoints detected by the feature detector
  std::vector<cv::KeyPoint> keypoints_;
  ///\brief set of descriptors extracted by the feature extractor
  cv::Mat descriptors_;

};

// Macro used to create definition of visual features implemented in opencv
// 'dName' is the feature name defined by opencv
#define DECLARE_VISUAL_FEATURE_CLASS(dName) \
class dName##VisualFeatureLandmark : public VisualFeatureLandmark { \
 public: \
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {  \
    return SemanticLandmarkPtr(new dName##VisualFeatureLandmark(image, pose)); \
  } \
  \
 protected:  \
  explicit dName##VisualFeatureLandmark(const cv::Mat& image, const SE3& pose);\
  \
}; \


DECLARE_VISUAL_FEATURE_CLASS(SIFT);
DECLARE_VISUAL_FEATURE_CLASS(ORB);
DECLARE_VISUAL_FEATURE_CLASS(SURF);

}

#endif // X_VIEW_VISUAL_FEATURE_LANDMARK_H_

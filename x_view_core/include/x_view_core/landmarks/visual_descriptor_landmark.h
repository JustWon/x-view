#ifndef X_VIEW_VISUAL_DESCRIPTOR_LANDMARK_H_
#define  X_VIEW_VISUAL_DESCRIPTOR_LANDMARK_H_

#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

namespace x_view {

/**
 * \brief A VisualDescriptorLandmark is a landmark containing descriptors
 * typically extracted in classic computer vision algorithms such as SIFT, ORB etc.
 * This is an abstract class (interface) implemented by all landmarks based
 * on classical visual descriptors.
 */
class VisualDescriptorLandmark : public AbstractSemanticLandmark {

 public:
  explicit VisualDescriptorLandmark(const cv::Mat& image, const SE3& pose);

  /// \brief Number of features to be extracted by the detector in case no
  /// parameter is provided.
  static int DEFAULT_NUM_VISUAL_FEATURES;
  /// \brief Hessian threshold value to use by the detector in case no
  /// parameter is provided.
  static float DEFAULT_HESSIAN_THRESHOLD;

  /// \brief Class responsible for extracting visual features.
  std::unique_ptr<cv::Feature2D> features_extractor;
};

// Macro used to create definition of visual features implemented in opencv
// 'dName' is the feature name defined by opencv.
#define DECLARE_VISUAL_FEATURE_CLASS(dName) \
class dName##VisualDescriptorLandmark : public VisualDescriptorLandmark { \
 public: \
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {  \
    return std::make_shared<dName##VisualDescriptorLandmark>(dName##VisualDescriptorLandmark(image, pose)); \
  } \
  \
 protected:  \
  explicit dName##VisualDescriptorLandmark(const cv::Mat& image, const SE3& pose);\
  \
}; \


DECLARE_VISUAL_FEATURE_CLASS(SIFT);
DECLARE_VISUAL_FEATURE_CLASS(ORB);
DECLARE_VISUAL_FEATURE_CLASS(SURF);

#undef DECLARE_VISUAL_FEATURE_CLASS

}

#endif // X_VIEW_VISUAL_DESCRIPTOR_LANDMARK_H_

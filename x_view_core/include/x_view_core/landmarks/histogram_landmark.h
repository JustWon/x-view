#ifndef X_VIEW_HISTOGRAM_LANDMARK_H_
#define  X_VIEW_HISTOGRAM_LANDMARK_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>

namespace x_view {

/**
 * \brief A HistogramLandmark represents a landmark by counting how many
 * pixels vote for each semantic class
 */
class HistogramLandmark : public AbstractSemanticLandmark {

 public:
  // FIXME: the histogram representation must be stored in feature_
  ///\brief histogram of size \f$N\f$ where \f$N\f$ is the number of possible
  /// semantic classes contained/supported by the dataset
  cv::Mat histogram_;

  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {
    return SemanticLandmarkPtr(new HistogramLandmark(image, pose));
  }

 protected:
  explicit HistogramLandmark(const cv::Mat& image, const SE3& pose);

};

}

#endif // X_VIEW_HISTOGRAM_LANDMARK_H_

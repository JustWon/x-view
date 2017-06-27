#ifndef X_VIEW_HISTOGRAM_LANDMARK_H_
#define  X_VIEW_HISTOGRAM_LANDMARK_H_

#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief A HistogramLandmark represents a landmark by counting how many
 * pixels vote for each semantic class.
 */
class HistogramLandmark : public AbstractSemanticLandmark {

 public:
  static SemanticLandmarkPtr create(const FrameData& frame_data) {
    return std::make_shared<HistogramLandmark>(HistogramLandmark(frame_data));
  }

 protected:
  /**
   * \brief Computes a histogram of semantic label frequencies encoded in the
   * first channel of the image passed as argument.
   * \param image Semantic image storing semantic class in the first channel
   * of each pixel.
   * \param pose Robot's pose.
   */
  explicit HistogramLandmark(const FrameData& frame_data);

};

}

#endif // X_VIEW_HISTOGRAM_LANDMARK_H_

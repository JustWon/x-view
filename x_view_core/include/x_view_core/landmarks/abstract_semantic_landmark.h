#ifndef X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H
#define X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <memory>

namespace x_view {

/**
 * \brief Internal representation of a semantic landmark
 * \note The effective implementation of a semantic landmark might be a subclass of XViewSemantics
 */
class AbstractSemanticLandmark {

 public:
  /**
   * \brief When a landmark is initialized, it must directly
   * \param image
   * \param pose
   */
  AbstractSemanticLandmark(const cv::Mat& image, const SE3& pose);
  virtual ~AbstractSemanticLandmark();

  /// \brief Image given as input for the landmark
  const cv::Mat image_;

  /// \brief Robot's pose associated to this semantic landmark
  const SE3 pose_;

}; // struct AbstractSemanticLandmark

}
#endif //X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#ifndef X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H
#define X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Internal representation of a semantic landmark. Each landmark type
 * used in XView must implement this interface
 * \details A SemanticLandmark is an object created whenever new semantic
 * data is given to XView. In particular a semanticLandmark might contain
 * data about the robot's pose and an internal semantic representation of
 * what the robot experiences in that moment.
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

  /// \brief Returns a const reference to the stored feature representation
  const ConstFeaturePtr& getFeature() const {
    return feature_;
  }

 protected:
  /// \brief internal representation of features extracted in this landmark
  ConstFeaturePtr feature_;

}; // AbstractSemanticLandmark

}
#endif //X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

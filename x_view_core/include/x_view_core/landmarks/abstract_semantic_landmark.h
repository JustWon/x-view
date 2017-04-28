#ifndef X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H
#define X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#include <opencv2/core/core.hpp>

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief Internal representation of a semantic landmark. Each landmark type
 * used in XView must implement this interface.
 * \details A SemanticLandmark is an object created whenever new semantic
 * data is given to XView. In particular a semanticLandmark might contain
 * data about the robot's pose and an internal semantic representation of
 * what the robot experiences in that moment.
 */
class AbstractSemanticLandmark {

 public:
  /**
   * \brief When a landmark is initialized, it must directly compute its
   * internal representation.
   * \param image Semantic image associated to the landmark.
   * \param pose Robot's pose.
   */
  AbstractSemanticLandmark(const cv::Mat& image, const SE3& pose);
  virtual ~AbstractSemanticLandmark();

  /// \brief Returns a const reference to the image associated with this
  /// landmark.
  const cv::Mat& getSemanticImage() const;

  /// \brief Returns a const reference to the robot's pose associated with
  /// this landmark.
  const SE3& getPose() const;

  /// \brief Returns a const reference to the stored descriptor representation.
  const ConstDescriptorPtr& getDescriptor() const;

 protected:
  /// \brief Semantic image given as input for the landmark.
  const cv::Mat semantic_image_;

  /// \brief Robot's pose associated to this semantic landmark.
  const SE3 pose_;

  /// \brief internal representation of descriptor extracted in this landmark.
  ConstDescriptorPtr descriptor_;

}; // AbstractSemanticLandmark

}
#endif //X_VIEW_ABSTRACT_SEMANTIC_LANDMARK_H

#ifndef X_VIEW_SEMANTIC_LANDMARK_FACTORY_H
#define X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Class responsible for creating new landmarks
 */
class SemanticLandmarkFactory {

 public:

  /// function pointer passed to the factory to create new landmarks
  typedef SemanticLandmarkPtr (*CreateCallBack)(const cv::Mat&, const SE3&);

  /**
   * \brief Sets the landmark type the factory is going to generate
   * \param type the type of semantic landmark one wants to generate through
   * the factory
   */
  static void setCreatorFunction(CreateCallBack cb);

  /**
   * \brief Function exposed to the user to create new semantic landmark objects
   * \param image Image containing semantic segmentation
   * \param pose  3D pose of the robot associated to the image
   * \return landmark pointer to abstract base landmark class which is filled
   * up with a concrete landmark type
   */
  static SemanticLandmarkPtr createSemanticLandmark(const cv::Mat& image, const SE3& pose);

 private:
  static CreateCallBack cb_;

};
}

#endif //X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

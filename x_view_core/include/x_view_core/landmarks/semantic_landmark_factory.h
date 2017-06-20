#ifndef X_VIEW_SEMANTIC_LANDMARK_FACTORY_H
#define X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Class responsible for creating new landmarks.
 */
class SemanticLandmarkFactory {

 public:

  /// \brief Function pointer passed to the factory to create new landmarks.
  typedef SemanticLandmarkPtr (* CreateCallBack)(const cv::Mat&, const SE3&);

  /**
   * \brief Sets the landmark type the factory is going to generate.
   * \param callback Function pointer called to create a new landmark.
   */
  static void setCreatorFunction(CreateCallBack callback);

  /**
   * \brief Function exposed to the user to create new semantic landmark
   * objects.
   * \param image Image containing semantic information on the first channel.
   * \param pose  3D pose of the robot associated to the image.
   * \return landmark Pointer to abstract base landmark class which is filled
   * up with a concrete landmark instance.
   */
  static SemanticLandmarkPtr createSemanticLandmark(const cv::Mat& image,
                                                    const SE3& pose);

 private:
  ///\brief A function pointer to the static function responsible for
  /// creating new semantic landmarks given an image and a pose.
  static CreateCallBack callback_;

};
}

#endif //X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

#ifndef X_VIEW_SEMANTIC_LANDMARK_FACTORY_H
#define X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

namespace x_view {

// forward declaration
struct AbstractSemanticLandmark;

// parameters used by the landmarks
struct AbstractSemanticLandmarkParameters {
  AbstractSemanticLandmarkParameters(){}
};

/**
 * \brief Class responsible for creating new landmarks
 */
class SemanticLandmarkFactory {

 public:

  enum SEMANTIC_LANDMARK_TYPE {
    UNDEFINED_SEMANTIC_LANDMARK_TYPE = -1,
    ORB_VISUAL_FEATURE,
    NUM_SEMANTIC_LANDMARK_TYPES
  };

  /**
   * \brief Creates a factory object which will create semantic landmarks defined by the passed argument
   * \param type enum specifying the semantic landmark type to be constructed
   */
  explicit SemanticLandmarkFactory(SEMANTIC_LANDMARK_TYPE type = SEMANTIC_LANDMARK_TYPE::UNDEFINED_SEMANTIC_LANDMARK_TYPE)
      : semanticLandmarkType_(type) {}

  void setSemanticLandmarkType(const SEMANTIC_LANDMARK_TYPE type) {
    semanticLandmarkType_ = type;
  }

  SEMANTIC_LANDMARK_TYPE getSemanticLandmarkType() const {
    return semanticLandmarkType_;
  }

  /**
   * \brief Function exposed to the user to create new semantic landmark objects
   * \param image Image containing semantic segmentation
   * \param pose  3D pose of the robot associated to the image
   * \return landmark pointer to abstract base landmark class which is filled
   * up with a concrete landmark type
   */
  SemanticLandmarkPtr createSemanticLandmark(const cv::Mat& image, const SE3& pose);

 private:
  SEMANTIC_LANDMARK_TYPE semanticLandmarkType_;

};
}

#endif //X_VIEW_SEMANTIC_LANDMARK_FACTORY_H

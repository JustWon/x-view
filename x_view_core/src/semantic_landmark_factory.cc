#include <x_view_core/semantic_landmark_factory.h>

#include <x_view_core/abstract_semantic_landmark.h>
#include <x_view_core/orb_visual_feature.h>

namespace x_view {

SemanticLandmarkPtr SemanticLandmarkFactory::
createSemanticLandmark(const cv::Mat& image, const SE3& pose) {
  CHECK(this->semanticLandmarkType_ >= 0 &&
      this->semanticLandmarkType_
          < SEMANTIC_LANDMARK_TYPE::NUM_SEMANTIC_LANDMARK_TYPES);

  switch (this->semanticLandmarkType_) {
    case SEMANTIC_LANDMARK_TYPE::ORB_VISUAL_FEATURE : {
      // FIXME: in case of this feature we need to know also how many
      // features we need to detect for each image
      int num_desired_visual_features = 500;
      return SemanticLandmarkPtr(new ORBVisualFeature(image, pose, num_desired_visual_features));
    }
    case SEMANTIC_LANDMARK_TYPE::UNDEFINED_SEMANTIC_LANDMARK_TYPE : {
      throw std::runtime_error(
          "Make sure to set a valid landmark type in the landmark factory");
    }
    default : {
      throw std::runtime_error(
          "Unrecognized semantic landmark type in semantic landmark factory");
    }
  }
}

}

#include <x_view_core/semantic_landmark_factory.h>

#include <x_view_core/abstract_semantic_landmark.h>
#include <x_view_core/bos.h>
#include <x_view_core/graph.h>

#include <opencv2/features2d/features2d.hpp>

#include <exception>

namespace x_view {

SemanticLandmarkPtr SemanticLandmarkFactory::createSemanticLandmark(const cv::Mat& image,
                                                     const SE3& pose) {
  CHECK(this->semanticLandmarkType_ >= 0 &&
      this->semanticLandmarkType_
          < SEMANTIC_LANDMARK_TYPE::NUM_SEMANTIC_LANDMARK_TYPES);

  switch (this->semanticLandmarkType_) {
    case SEMANTIC_LANDMARK_TYPE::BOS_HISTOGRAM : {
      return SemanticLandmarkPtr(new BoSHistogram(image, pose));
    }
    case SEMANTIC_LANDMARK_TYPE::BOS_VISUAL_FEATURE : {
      // FIXME: in case of this feature we need to know also how many
      // features we need to detect for each image
      return SemanticLandmarkPtr(new BoSVisualFeatures(image, pose, 200));
    }
    case SEMANTIC_LANDMARK_TYPE::GRAPH : {
      return SemanticLandmarkPtr(new SemanticGraph(image, pose));
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

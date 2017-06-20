#include <x_view_core/landmarks/semantic_landmark_factory.h>

namespace x_view {

SemanticLandmarkFactory::CreateCallBack SemanticLandmarkFactory::callback_;

void SemanticLandmarkFactory::setCreatorFunction(CreateCallBack callback) {
  callback_ = callback;
}

SemanticLandmarkPtr SemanticLandmarkFactory::createSemanticLandmark(const cv::Mat& image,
                                                                    const SE3& pose) {

  return callback_(image, pose);
}

}

#include <x_view_core/landmarks/semantic_landmark_factory.h>

namespace x_view {

SemanticLandmarkFactory::CreateCallBack SemanticLandmarkFactory::cb_;

void SemanticLandmarkFactory::setCreatorFunction(CreateCallBack cb) {
  cb_ = cb;
}

SemanticLandmarkPtr SemanticLandmarkFactory::createSemanticLandmark(const cv::Mat& image,
                                                                    const SE3& pose) {

  return cb_(image, pose);
}

}

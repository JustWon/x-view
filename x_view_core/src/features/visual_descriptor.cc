#include <x_view_core/features/visual_descriptor.h>

namespace x_view {

VisualDescriptor::VisualDescriptor(const cv::Mat& descriptor,
                             const KeyPointsVector& keypoints)
    : VectorDescriptor(descriptor),
      key_points_vector_(keypoints) {
}

VisualDescriptor::~VisualDescriptor() {
}

const VisualDescriptor::KeyPointsVector& VisualDescriptor::getKeyPoints() const {
  return key_points_vector_;
}

}

#include <x_view_core_os/features/vector_descriptor.h>

namespace x_view {

VectorDescriptor::VectorDescriptor(const cv::Mat& descriptor)
    : AbstractDescriptor(),
      descriptor_(descriptor) {
}

VectorDescriptor::~VectorDescriptor() {
}

int VectorDescriptor::descriptorDimension() const {
  return descriptor_.cols;
}

int VectorDescriptor::numFeatures() const {
  return descriptor_.rows;
}

}

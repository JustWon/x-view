#include <x_view_core/features/vector_feature.h>

namespace x_view {

VectorFeature::VectorFeature(const cv::Mat& feature)
    : AbstractFeature(),
      feature_(feature) {
}

VectorFeature::~VectorFeature() {
}

int VectorFeature::featureDimension() const {
  return feature_.cols;
}

int VectorFeature::numFeatures() const {
  return feature_.rows;
}

}

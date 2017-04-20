#include <x_view_core/features/vector_feature.h>

namespace x_view {

/////////////////////////////// CV::MAT-based //////////////////////////////////

template<>
int CVMatFeature::featureDimension() const {
  return feature_repr_.cols;
}
template<>
int CVMatFeature::numFeatures() const {
  return feature_repr_.rows;
}



///////////////////////////// std::vec-based  //////////////////////////////////

template<>
int IntVecFeature::featureDimension() const {
  return (int) feature_repr_.size();
}
template<>
int IntVecFeature::numFeatures() const {
  return 1;
}

}

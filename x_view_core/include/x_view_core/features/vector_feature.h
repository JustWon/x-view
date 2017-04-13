#ifndef X_VIEW_VECTOR_FEATURE_H
#define X_VIEW_VECTOR_FEATURE_H

#include <x_view_core/features/abstract_feature.h>

#include <opencv2/core/core.hpp>

#include <vector>

namespace x_view {

/**
 * \brief This class encapsulates all types of features that might be
 * represented as vector or as a matrix.
 * \tparam Container the way the feature is represented internally
 */
template<typename Container>
class VectorFeature : public AbstractFeature {

 public:

  typedef Container FeatureRepr;

  VectorFeature(const FeatureRepr& feature)
      : AbstractFeature(),
        feature_repr_(feature) {}

  virtual ~VectorFeature() {}

  const FeatureRepr& getFeature() const {
    return feature_repr_;
  }

  ///\brief returns the dimensionality of the feature(s) stored in this object
  int featureDimension() const;
  ///\brief returns the number of independent features stored in this object
  int numFeatures() const;

 protected:
  const FeatureRepr feature_repr_;
};


/////////////////////////////// CV::MAT-based //////////////////////////////////

// Features extracted from images such as SIFT/SURF/ORB are internally
// represented as a cv::Mat
typedef VectorFeature<cv::Mat> CVMatFeature;
int CVMatFeature::featureDimension() const {
  return feature_repr_.cols;
}
int CVMatFeature::numFeatures() const {
  return feature_repr_.rows;
}

///////////////////////////// std::vec-based  //////////////////////////////////

// A histogram feature only needs a vector of integers (votes) as representation
typedef VectorFeature<std::vector<int>> IntVecFeature;
int IntVecFeature::featureDimension() const {
  return feature_repr_.size();
}
int IntVecFeature::numFeatures() const {
  return 1;
}

}

#endif //X_VIEW_VECTOR_FEATURE_H

#ifndef X_VIEW_VECTOR_FEATURE_H
#define X_VIEW_VECTOR_FEATURE_H

#include <x_view_core/features/abstract_feature.h>

#include <opencv2/core/core.hpp>

#include <vector>

namespace x_view {

/**
 * \brief This class encapsulates all types of features that might be
 * represented as vector or as a matrix.
 */
class VectorFeature : public AbstractFeature {

 public:

  VectorFeature(const cv::Mat& feature);

  virtual ~VectorFeature();

  ///\brief returns a const reference of the feature representation
  const cv::Mat& getFeature() const {
    return feature_;
  }

  ///\brief returns the dimensionality of the feature(s) stored in this object
  int featureDimension() const;
  ///\brief returns the number of independent features stored in this object
  int numFeatures() const;

 protected:
  const cv::Mat feature_;
};

}

#endif //X_VIEW_VECTOR_FEATURE_H

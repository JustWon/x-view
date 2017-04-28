#ifndef X_VIEW_VECTOR_DESCRIPTOR_H
#define X_VIEW_VECTOR_DESCRIPTOR_H

#include <vector>

#include <opencv2/core/core.hpp>

#include <x_view_core/features/abstract_descriptor.h>

namespace x_view {

/**
 * \brief This class encapsulates all types of descriptors that might be
 * represented as vector or as a matrix.
 */
class VectorDescriptor : public AbstractDescriptor {

 public:

  VectorDescriptor(const cv::Mat& feature);

  virtual ~VectorDescriptor();

  /// \brief Returns a const reference of the descriptor representation.
  const cv::Mat& getDescriptor() const {
    return descriptor_;
  }

  /// \brief Returns the dimensionality of the descriptor stored in this object.
  int descriptorDimension() const;
  /// \brief Returns the number of independent features stored in this object.
  int numFeatures() const;

 protected:
  const cv::Mat descriptor_;
};

}

#endif //X_VIEW_VECTOR_DESCRIPTOR_H

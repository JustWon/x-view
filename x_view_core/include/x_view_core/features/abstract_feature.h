#ifndef X_VIEW_ABSTRACT_FEATURE_H
#define X_VIEW_ABSTRACT_FEATURE_H

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief Internal representation of a feature. Each feature used in XView
 * must implement this interface.
 * \details The introduction of AbstractFeatures allow XView to operate
 * seamlessly with different features such as 'vector-based' features
 * (histograms, visual features, etc) an with more complex ones such as
 * 'graph-based' features.
 */
class AbstractFeature {

 public:
  AbstractFeature();
  // trick to make AbstractFeature a pure virtual class with no need to
  // introduce useless abstract methods which must be implemented in the
  // subclass
  virtual ~AbstractFeature() = 0;

}; // AbstractFeature


}
#endif //X_VIEW_ABSTRACT_FEATURE_H

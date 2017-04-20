#ifndef X_VIEW_GRAPH_FEATURE_H
#define X_VIEW_GRAPH_FEATURE_H

#include <x_view_core/features/abstract_feature.h>

namespace x_view {


// FIXME: this is just a name placeholder for the way we are going to define
// a graph
class GraphRepresentationObject{

};

/**
 * \brief This class encapsulates all types of features that might be
 * represented as graph.
 */
class GraphFeature : public AbstractFeature {

 public:

  typedef GraphRepresentationObject FeatureRepr;

  GraphFeature(const FeatureRepr& feature)
      : AbstractFeature(),
        feature_repr_(feature) {}

  virtual ~GraphFeature() {}

  const GraphFeature& getFeature() const {
    return feature_repr_;
  }

  int numNodes() const {
    // return feature_repr_.numNodes();
  }

 protected:
  const FeatureRepr feature_repr_;
};

}

#endif //X_VIEW_GRAPH_FEATURE_H

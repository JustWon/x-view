#ifndef X_VIEW_GRAPH_DESCRIPTOR_H
#define X_VIEW_GRAPH_DESCRIPTOR_H

#include <x_view_core/features/abstract_descriptor.h>
#include <x_view_core/features/graph.h>

namespace x_view {

/**
 * \brief This class encapsulates all types of descriptors that might be
 * represented as graph.
 */
class GraphDescriptor : public AbstractDescriptor {

 public:
  // a graph is represented by the Graph class
  typedef Graph DescriptorRepr;

  GraphDescriptor(const DescriptorRepr& descriptor);
  virtual ~GraphDescriptor() {}

  const DescriptorRepr& getDescriptor() const {
    return descriptor_;
  }

 protected:
  const DescriptorRepr descriptor_;
};

}

#endif //X_VIEW_GRAPH_DESCRIPTOR_H

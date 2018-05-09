#ifndef X_VIEW_GRAPH_FACTORY_H
#define X_VIEW_GRAPH_FACTORY_H

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/semantic_graph.h>

namespace x_view {

class GraphFactory {
  GraphFactory(const SemanticGraph& graph);
  ~GraphFactory();

  void merge_graphs(const SemanticGraph& graph);

 private:
    SemanticGraph merged_graph_;
};
}

#endif //X_VIEW_GRAPH_FACTORY_H
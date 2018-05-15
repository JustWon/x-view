#ifndef X_VIEW_GRAPH_GRAPH_MERGER_2_
#define X_VIEW_GRAPH_GRAPH_MERGER_2_

#include <x_view_core_os/landmarks/semantic_graph.h>
#include <x_view_core_os/x_view_types.h>

namespace x_view {

class GraphMerger2 {

 public:
  /**
   * \brief This object takes vector of individual graphs and merges them.
   */
  GraphMerger2();

  ~GraphMerger2();

  void mergeGraphs(std::vector<SemanticGraph>& graphs, Graph*
  out_graph);

  void mergeGraphs(const Graph& graph_a, Graph* out_graph);

//  void getMergedGraph(SemanticGraph* merged_graph);

// private:
//  SemanticGraph merged_graph_;
};
}

#endif // X_VIEW_GRAPH_GRAPH_MERGER_2_
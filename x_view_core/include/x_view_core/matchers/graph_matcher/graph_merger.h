#ifndef X_VIEW_GRAPH_MERGER_H
#define X_VIEW_GRAPH_MERGER_H

#include <x_view_core/features/graph.h>
#include <x_view_core/matchers/graph_matcher/graph_matcher.h>


#include <queue>

namespace x_view {

class GraphMerger {

  typedef std::queue<VertexDescriptor> DescriptorQueue;

 public:

  GraphMerger(const Graph& database_graph, const Graph& query_graph,
              const GraphMatcher::GraphMatchingResult& matching_result);

  Graph getMergedGraph();

 private:
  const Graph& database_graph_;
  const Graph& query_graph_;
  const GraphMatcher::GraphMatchingResult & matching_result_;

  Graph merged_graph_;
  DescriptorQueue still_to_process_;
  std::unordered_map<VertexDescriptor, VertexDescriptor> query_in_db_;
  int current_vertex_index_;

  std::vector<VertexDescriptor> matched_vertices_;

  GraphMatcher::MaxSimilarityMatrixType computeAgreementMatrix() const;

  void addVertexToMergedGraph(const VertexDescriptor& source_in_query_graph);

};

}

#endif //X_VIEW_GRAPH_MERGER_H

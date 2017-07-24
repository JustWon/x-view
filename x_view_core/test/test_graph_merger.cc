#include "test_graph_merger.h"

#include <x_view_core/matchers/graph_matcher.h>

namespace x_view_test {

void mergeGraphs(const Graph& g1, const Graph g2, Graph* merged) {

  CHECK_NOTNULL(merged);

  // Define the parameters needed to match the graphs.
  RandomWalkerParams random_walker_params;
  random_walker_params.num_walks = 1000;
  random_walker_params.walk_length = 3;
  random_walker_params.random_sampling_type =
      RandomWalkerParams::SAMPLING_TYPE::AVOIDING;

  VertexSimilarity::SCORE_TYPE score_type =
      VertexSimilarity::SCORE_TYPE::WEIGHTED;

  // Compute the similarities between the graphs via the GraphMatcher.
  GraphMatcher graph_matcher(random_walker_params, score_type);

  // Add the first graph to the matcher.
  graph_matcher.addDescriptor(g1);

  // Match the second graph to the first one and get the matching result.
  const auto matching_result =
      std::dynamic_pointer_cast<GraphMatcher::GraphMatchingResult>(
          graph_matcher.match(g2));

  // Verify that the cast was successful
  CHECK_NOTNULL(matching_result.get());


  // Define the parameters to merge the graphs.
  GraphMergerParameters graph_merger_parameters;
  graph_merger_parameters.time_window = std::numeric_limits<uint64_t>::max();
  graph_merger_parameters.similarity_threshold = 0.f;
  graph_merger_parameters.distance_threshold =
      std::numeric_limits<float>::max();
  GraphMerger graph_merger(g1, g2, *(matching_result.get()),
                           graph_merger_parameters);
  *merged = graph_merger.computeMergedGraph();
}

}

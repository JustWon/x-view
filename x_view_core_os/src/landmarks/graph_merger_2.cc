#include <x_view_core_os/features/graph.h>
#include <x_view_core_os/landmarks/graph_landmark/graph_landmark.h>
#include <x_view_core_os/landmarks/graph_merger_2.h>

#include <nabo/nabo.h>

namespace x_view {

GraphMerger2::GraphMerger2() {};

GraphMerger2::~GraphMerger2() {};

void GraphMerger2::mergeGraphs(std::vector<SemanticGraph> &graphs, Graph *
out_graph) {

  // Set initial graph.
  Graph graph_query;
  graphs[0].getDescriptor(out_graph);
  for (size_t i = 1u; i < graphs.size(); ++i) {
    // Apply 3D space merging strategy.

    graphs[i].getDescriptor(&graph_query);
    const uint64_t num_query_vertices = boost::num_vertices(graph_query);

    auto vertices_query = boost::vertices(graph_query);
    auto vertices_db = boost::vertices(*out_graph);

    for (auto q = vertices_query.first; q != vertices_query.second; ++q) {
      int iter = 0;
      const uint64_t num_db_vertices = boost::num_vertices(*out_graph);
      for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
        ++iter;
        // Check for the same label of vertices.
        if (graph_query[*q].semantic_label == (*out_graph)[*d]
            .semantic_label) {
          // Check if location is within the merging threshold.
          // todo (gawela): Make the merging threshold parametric.
          if ((graph_query[*q].location_3d - (*out_graph)[*d].location_3d)
              .norm()
              < 7.5) {
            // Update observers.
            (*out_graph)[*d].observers.push_back(graph_query[*q]
                                                     .observers[0]);

            break;
          }
        }
        // Add remaining vertex to graph.
        if (iter == num_db_vertices) {
          boost::add_vertex(graph_query[*q], (*out_graph));
        }
      }
    }
  }

  // todo(gawela): Check that merged_graph_ is actually growing.

  // Create matrix of vertex locations for nearest neighbor search.
  auto vertices_db = boost::vertices(*out_graph);
  const uint64_t num_db_vertices = boost::num_vertices(*out_graph);
  Eigen::MatrixXf locations(3, num_db_vertices);

  int iter = 0;
  for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
    locations.block(0, iter, 3, 1) = (*out_graph)[*d].location_3d
        .cast<float>();
    ++iter;
  }

  // Use KDTree to establish edges between nodes.
  Nabo::NNSearchF *nns = Nabo::NNSearchF::createKDTreeLinearHeap(locations);
  // todo(gawela): Make k parametric.
  const int k = 5;
  Eigen::VectorXi indices(k);
  Eigen::VectorXf dists2(k);
  iter = 0;
  for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
    nns->knn((*out_graph)[*d].location_3d.cast<float>(), indices, dists2, k);
    for (size_t i = 0u; i < indices.rows(); ++i) {
      // Add edges for all nearest neighbours within threshold distance.
      // todo(gawela): Make dist threshold parametric (square!!), and also
      // num_times_seen.
      int num_times_seen = 1;
      if (dists2(i) < 100.0) {
        boost::add_edge(iter, indices(i), {iter, indices(i), num_times_seen},
                        (*out_graph));
      }
    }
    ++iter;
  }
}

void GraphMerger2::mergeGraphs(const Graph &graph_a, Graph *out_graph) {

  auto vertices_query = boost::vertices(graph_a);
  auto vertices_db = boost::vertices(*out_graph);

  if (boost::num_vertices(*out_graph) > 0) {
    // Apply 3D space merging strategy.

    const uint64_t num_query_vertices = boost::num_vertices(graph_a);
    std::cout << "Num q vertices " << num_query_vertices << std::endl;

    for (auto q = vertices_query.first; q != vertices_query.second; ++q) {
      int iter = 0;
      const uint64_t num_db_vertices = boost::num_vertices(*out_graph);
      for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
        ++iter;
        // Check for the same label of vertices.
        if (graph_a[*q].semantic_label == (*out_graph)[*d]
            .semantic_label) {
          // Check if location is within the merging threshold.
          // todo (gawela): Make the merging threshold parametric.
          if ((graph_a[*q].location_3d - (*out_graph)[*d].location_3d)
              .norm()
              < 7.5) {
            // Update observers.
            std::cout << "Merging vertex." << std::endl;
            (*out_graph)[*d].observers.push_back(graph_a[*q].observers[0]);
            break;
          }
        }
        // Add remaining vertex to graph.
        if (iter == num_db_vertices) {
          std::cout << "Adding leftover vertex." << std::endl;
          boost::add_vertex(graph_a[*q], (*out_graph));
        }
      }
    }
  } else {
    *out_graph = graph_a;
  }
  // Create matrix of vertex locations for nearest neighbor search.
  vertices_db = boost::vertices(*out_graph);
  const uint64_t num_db_vertices = boost::num_vertices(*out_graph);
  std::cout << "Num db vertices " << num_db_vertices << std::endl;
  Eigen::MatrixXf locations(3, num_db_vertices);

  int iter = 0;
  for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
    locations.block(0, iter, 3, 1) = (*out_graph)[*d].location_3d
        .cast<float>();
    ++iter;
  }

  // Use KDTree to establish edges between nodes.
  Nabo::NNSearchF *nns = Nabo::NNSearchF::createKDTreeLinearHeap(locations);
  // todo(gawela): Make k parametric.
  const int k = std::min(8, int(num_db_vertices - 1));
  Eigen::VectorXi indices(k);
  Eigen::VectorXf dists2(k);
  iter = 0;
  for (auto d = vertices_db.first; d != vertices_db.second; ++d) {
    nns->knn((*out_graph)[*d].location_3d.cast<float>(), indices, dists2, k);
    for (size_t i = 0u; i < indices.rows(); ++i) {
      // Add edges for all nearest neighbours within threshold distance.
      // todo(gawela): Make dist threshold parametric (square!!), and also
      // num_times_seen.
      int num_times_seen = 1;
      if (dists2(i) < 200.0) {
        boost::add_edge(iter, indices(i), {iter, indices(i), num_times_seen},
                        (*out_graph));
      }
    }
    ++iter;
  }
  // todo(gawela): Merge unconnected graphs.
}
}
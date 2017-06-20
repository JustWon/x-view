#include <x_view_core/landmarks/graph_landmark/graph_builder.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>

#include <boost/graph/connected_components.hpp>

namespace x_view {

std::vector<const Blob*> GraphBuilder::DEFAULT_BLOB_VECTOR;

Graph GraphBuilder::createGraphFromNeighborBlobs(const ImageBlobs& blobs,
                                                 const GraphBuilderParams& params) {

  Graph graph;

  // Vector containing references to the created graph nodes.
  std::vector<VertexDescriptor> vertex_descriptors;
  // Vector keeping track of which blob is associated to which graph node.
  std::vector<const Blob*> blob_vector;

  // Add all blobs to the graph.
  GraphBuilder::addBlobsToGraph(blobs, &graph, &vertex_descriptors,
                                &blob_vector);


  // create the edges between nodes sharing an edge
  for (int i = 0; i < blob_vector.size(); ++i) {
    for (int j = i + 1; j < blob_vector.size(); ++j) {

      const Blob* bi = blob_vector[i];
      const Blob* bj = blob_vector[j];

      // only create an edge between the two blobs if they are neighbors
      if (Blob::areNeighbors(*bi, *bj, params.max_distance_for_neighborhood))
        boost::add_edge(vertex_descriptors[i], vertex_descriptors[j],
                        {i, j}, graph);
    }
  }

  // Check that the generated graph is a single connected component.
  std::vector<int> component(boost::num_vertices(graph));
  int num_components = boost::connected_components(graph, &component[0]);
  if (num_components == 1)
    return graph;
  else {
    LOG(WARNING) << "Graph built upon semantic image presents "
                 << num_components << " disconnected components.";
    connectClosestVerticesOfDisconnectedGraph(&graph, component);
    num_components = boost::connected_components(graph, &component[0]);
    CHECK_EQ(num_components, 1)
      << "The graph resulting from connectClosestVerticesOfDisconnectedGraph "
      << "has " << num_components << " components.";
    return graph;
  }

}

Graph GraphBuilder::createCompleteGraph(const ImageBlobs& blobs) {

  Graph graph;

  std::vector<VertexDescriptor> vertex_descriptors;

  GraphBuilder::addBlobsToGraph(blobs, &graph, &vertex_descriptors);


  // create the edges between the nodes
  for (int i = 0; i < vertex_descriptors.size(); ++i) {
    for (int j = i + 1; j < vertex_descriptors.size(); ++j) {
      boost::add_edge(vertex_descriptors[i], vertex_descriptors[j],
                      {i, j}, graph);
    }
  }

  return graph;
}

void GraphBuilder::addBlobsToGraph(const ImageBlobs& blobs,
                                   Graph* graph,
                                   std::vector<VertexDescriptor>* vertex_descriptors,
                                   std::vector<const Blob*>* blob_vector) {

  CHECK_NOTNULL(graph);
  CHECK_NOTNULL(vertex_descriptors);
  CHECK_NOTNULL(blob_vector);

  vertex_descriptors->clear();
  blob_vector->clear();

  // Each blob is a graph node
  int blob_count = 0;
  for (int c = 0; c < blobs.size(); ++c) {
    for (const Blob& blob : blobs[c]) {
      blob_vector->push_back(&blob);
      VertexProperty vertex =
          GraphBuilder::blobToGraphVertex(blob_count++, blob);
      vertex_descriptors->push_back(boost::add_vertex(vertex, *graph));
    }
  }

}

VertexProperty GraphBuilder::blobToGraphVertex(const int index,
                                               const Blob& blob) {
  const int semantic_label = blob.semantic_label;
  const std::string label = global_dataset_ptr->label(semantic_label);
  const int size = blob.num_pixels;
  const cv::Point center = blob.center;
  return VertexProperty{index, semantic_label, label, size, center};
}

void GraphBuilder::connectClosestVerticesOfDisconnectedGraph(Graph* graph,
                                                             const std::vector<
                                                                 int>& component) {
  std::vector<int> unique_components(component);
  std::sort(unique_components.begin(), unique_components.end());
  unique_components.erase(std::unique(unique_components.begin(),
                                      unique_components.end()),
                          unique_components.end());

  const unsigned long num_vertices = boost::num_vertices(*graph);

  auto dist_square = [](const cv::Point& p1, const cv::Point& p2) {
    cv::Point d(p1 - p2);
    return d.x * d.x + d.y * d.y;
  };
  // Iterate over each pair of components and determine the closest pair of
  // vertices to be connected.
  for (auto first_component = unique_components.begin();
       first_component != unique_components.end(); ++first_component) {
    const int first_component_id = *first_component;
    for (auto second_component = std::next(first_component);
         second_component != unique_components.end(); ++second_component) {
      const int second_component_id = *second_component;
      int min_distance_square = std::numeric_limits<int>::max();
      EdgeProperty closest_v_d_pair = {-1, -1};
      for (int i = 0; i < num_vertices; ++i) {
        if (component[i] == first_component_id) {
          const cv::Point& center_i = (*graph)[i].center;
          for (int j = 0; j < num_vertices; ++j) {
            if (component[j] == second_component_id) {
              const cv::Point& center_j = (*graph)[j].center;
              int dist2 = dist_square(center_i, center_j);
              if (dist2 < min_distance_square) {
                min_distance_square = dist2;
                closest_v_d_pair.from = i;
                closest_v_d_pair.to = j;
              }
            }
          }
        }
      }
      CHECK(closest_v_d_pair.from != -1 && closest_v_d_pair.to != -1)
      << "Function " << __FUNCTION__ << " could not determine which "
      << "vertices are the closest pair in the disconnected graph "
      << "between component " << first_component_id << " and "
      << second_component_id;
      // The closest vertices between first_component and second_component
      // are the ones contained in closest_v_d_pair.
      boost::add_edge(closest_v_d_pair.from, closest_v_d_pair.to,
                      closest_v_d_pair, *graph);
    }
  }
}

}

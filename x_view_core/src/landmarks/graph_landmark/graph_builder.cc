#include <x_view_core/landmarks/graph_landmark/graph_builder.h>

#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>

namespace x_view {

std::vector<const Blob*> GraphBuilder::DEFAULT_BLOB_VECTOR;

Graph GraphBuilder::createGraphFromNeighborBlobs(const ImageBlobs&
blobs, const GraphBuilderParams& params) {

  Graph graph;

  // vector containing references to the created graph nodes
  std::vector<VertexDescriptor> vertex_descriptors;
  // vector keeping track of which blob is associated to which graph node
  std::vector<const Blob*> blob_vector;

  // add all blobs to the graph
  GraphBuilder::addBlobsToGraph(blobs, &graph, &vertex_descriptors,
                                &blob_vector);


  // create the edges between nodes sharing an edge
  for (int i = 0; i < blob_vector.size(); ++i) {
    for (int j = i + 1; j < blob_vector.size(); ++j) {

      const Blob* bi = blob_vector[i];
      const Blob* bj = blob_vector[j];

      // only create an edge between the two blobs if they are neighbors
      if (Blob::areNeighbors(*bi, *bj, params.max_distance_for_neighborhood_))
        boost::add_edge(vertex_descriptors[i], vertex_descriptors[j],
                        {i, j}, graph);
    }
  }

  return graph;

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

  if (graph != nullptr) {
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
  } else {
    CHECK(false) << "Graph pointer passed to " << __FUNCTION__
                 << " is a nullptr";
  }

}

VertexProperty GraphBuilder::blobToGraphVertex(const int index,
                                                      const Blob& blob) {
  const int semantic_label = blob.semantic_label_;
  const std::string label = global_dataset_ptr->label(semantic_label);
  const int size = blob.num_pixels_;
  const cv::Point center = blob.center_;
  return VertexProperty{index, semantic_label, label, size, center};
}

}

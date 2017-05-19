#ifndef X_VIEW_GRAPH_BUILDER_H
#define X_VIEW_GRAPH_BUILDER_H

#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>
#include <x_view_core/features/graph_descriptor.h>

namespace x_view {

class GraphBuilder {
 public:
  static Graph::GraphType createGraphFromNeighborBlobs(const ImageBlobs& blobs);

  static Graph::GraphType createCompleteGraph(const ImageBlobs& blobs);

 private:

  // Dummy vector used as default argument for the the addBlobsToGraph function.
  static std::vector<const Blob*> DEFAULT_BLOB_VECTOR;

  /**
   * \brief Adds all blobs contained in the ImageBlobs datastructure to the
   * graph object passed as argument
   * \param blobs Datastructure containing all blobs.
   * \param graph Graph object to be filled up with new nodes.
   * \param vertex_descriptors Vector filled up with references to the
   * generated graph nodes.
   * \param blob_vector Vector containing pointers to the blobs associated to
   * the inserted nodes.
   */
  static void addBlobsToGraph(const ImageBlobs& blobs,
                              Graph::GraphType& graph,
                              std::vector<Graph::VertexDescriptor>&
                              vertex_descriptors,
                              std::vector<const Blob*>& blob_vector =
                              DEFAULT_BLOB_VECTOR);
  /**
   * \brief Generates a graph vertex containing the relevant information
   * extracted from the blob object passed as argument.
   * \param blob Blob object to the converted into a graph node.
   * \return A graph node containing the relevant information extracted from
   * the blob passed as argument.
   */
  static Graph::VertexProperty blobToGraphVertex(const Blob& blob);

};

}

#endif //X_VIEW_GRAPH_BUILDER_H

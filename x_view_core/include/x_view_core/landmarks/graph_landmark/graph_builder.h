#ifndef X_VIEW_GRAPH_BUILDER_H
#define X_VIEW_GRAPH_BUILDER_H

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>

namespace x_view {

/**
 * \brief Parameters used by the graph builder to build a graph from the blobs.
 */
struct GraphBuilderParams {
  GraphBuilderParams()
      : max_distance_for_neighborhood_(2) {}

  /// \brief Threshold for determining if two blobs are neighbors or not.
  int max_distance_for_neighborhood_;
};

class GraphBuilder {
 public:
  /**
   * \brief Creates a graph whose nodes correspond to the blobs contained in
   * the ImageBlobs datastructure passed as argument, and whose edges exist
   * between nodes corresponding to neighbor blobs.
   * \param blobs ImageBlobs datastructure containing all blobs.
   * \param params Parameters used by the graph builder during graph
   * construction.
   * \return A graph object containing nodes and edges based on the
   * ImageBlobs datastructure passed as argument.
   */
  static Graph::GraphType createGraphFromNeighborBlobs(const ImageBlobs&
  blobs, const GraphBuilderParams& params = GraphBuilderParams());

  /**
   * \brief Creates a complete graph whose nodes correspond to the blobs
   * contained in the ImageBlobs datastructure passed as argument.
   * \param blobs ImageBlobs datastructure containing all blobs.
   * \return A complete graph object containing nodes based on the ImageBlobs
   * datastructure passed as argument.
   */
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
                              Graph::GraphType* graph,
                              std::vector<Graph::VertexDescriptor>* vertex_descriptors,
                              std::vector<const Blob*>* blob_vector = &DEFAULT_BLOB_VECTOR);
  /**
   * \brief Generates a graph vertex containing the relevant information
   * extracted from the blob object passed as argument.
   * \param index Integer representing the index (position) of the blob
   * transformed into a vertex. This index represents corresponds to the
   * number of previously inserted blobs, i.e. the first inserted blob will
   * have index=0, the second index=1 etc.
   * \param blob Blob object to the converted into a graph node.
   * \return A graph node containing the relevant information extracted from
   * the blob passed as argument.
   */
  static Graph::VertexProperty blobToGraphVertex(const int index,
                                                 const Blob& blob);

};

}

#endif //X_VIEW_GRAPH_BUILDER_H

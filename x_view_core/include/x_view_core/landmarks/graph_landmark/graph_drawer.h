#ifndef X_VIEW_GRAPH_DRAWER_H
#define X_VIEW_GRAPH_DRAWER_H

#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace x_view {

class GraphDrawer {

 public:
  /**
   * \brief Prints the blob structure to the LOG(INFO).
   * \param blobs Blob datastructure to be printed.
   */
  static void printBlobs(const ImageBlobs& blobs);

  /**
   * \brief Generates a new image of size size representing the blobs
   * contained in the ImageBlobs datastructure passed as arguments with
   * additional information about the graph structure built upon it.
   * \param blobs ImageBlobs datastructure containing the blobs to be drawn.
   * \param graph Graph built upon the blob structure.
   * \param size Size og the image to be generated.
   * \return Image representing the blobs contained in the ImageBlobs
   * datastructure passed as argument and labels representing the graph
   * passed as argument.
   * \details This function is a utility function which internally calls
   * other drawing methods in the correct order, such that the resulting
   * image clearly shows all drawable data.
   */
  static cv::Mat createImageWithLabels(const ImageBlobs& blobs,
                                       const Graph& graph,
                                       const cv::Size& size);

  /**
   * \brief Generates a new image os size size representing the blobs
   * contained in the ImageBlobs datastructure passed as argument.
   * \param blobs ImageBlobs datastructure containing the blobs to be drawn.
   * \param size Size og the image to be generated.
   * \return Image representing the blobs contained in the ImageBlobs
   * datastructure passed as argument.
   */
  static cv::Mat createImageFromBlobs(const ImageBlobs& blobs,
                                      const cv::Size& size);

  /**
   * \brief Adds text/labels to the image positioned at the center of the
   * corresponding blob.
   * \param blobs ImageBlobs datastructure containing the labels to be rendered.
   * \param image The labels are rendered on top of the image passed as
   * argument.
   */
  static void addLabelsToImage(const ImageBlobs& blobs, cv::Mat* image);

  /**
   * \brief Adds ellipses to the image representing the blobs.
   * \param blobs ImageBlobs datastructure containing the labels to be rendered.
   * \param image The ellipses are rendered on top of the image passed as
   * argument.
   */
  static void addEllipsesToImage(const ImageBlobs& blobs, cv::Mat* image);

  /**
  * \brief Adds the graph nodes to the image representing the blobs.
  * \param graph Graph containing the nodes to be rendered.
  * \param image The nodes are rendered on top of the image passed as
  * argument.
  */
  static void addGraphNodesToImage(const Graph& graph, cv::Mat* image);

  /**
  * \brief Adds the graph edges to the image representing the blobs.
  * \param graph Graph containing the nodes to be rendered.
  * \param image The edges are rendered on top of the image passed as
  * argument.
  */
  static void addGraphEdgesToImage(const Graph& graph, cv::Mat* image);

};

}

#endif //X_VIEW_GRAPH_DRAWER_H

#ifndef X_VIEW_GRAPH_LANDMARK_H_
#define  X_VIEW_GRAPH_LANDMARK_H_

#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_core/features/graph_descriptor.h>
#include <x_view_core/features/graph.h>

namespace x_view {

/**
 * \brief A GraphLandmark represents a landmark by though a graph.
 */
class GraphLandmark : public AbstractSemanticLandmark {

 public:
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {
    return std::make_shared<GraphLandmark>(
        GraphLandmark(image, pose));
  }

  /// \brief A blob containing less pixel than 'MINIMUM_BLOB_SIZE' is ignored.
  static int MINIMUM_BLOB_SIZE;

  /// \brief Should the input image be preprocessed by a dilation and erosion
  /// procedure or not?
  static bool DILATE_AND_ERODE;

  // ******************************* ACCESS **********************************//

  const ImageBlobs& getBlobs() const {
    return image_blobs_;
  }

 protected:
  /**
   * \brief Performs a segmentation on the first channel the image passed as
   * argument to determine how semantic entities are represented in the scene.
   * A graph is built upon this segmentation defining its vertices as the
   * 'center of mass' of each segment of the image.
   * \param image Semantic image storing semantic class in the first channel
   * of each pixel.
   * \param pose Robot's pose.
   */
  explicit GraphLandmark(const cv::Mat& image, const SE3& pose);

 private:
  /**
   * \brief Structure containing the semantic segmentation of the
   * semantic_image_.
   * \details The dimensions of the structure are the following:
   * 'image_blobs_.size()' = dataset size (num semantic classes)
   * 'image_blobs_[i].size()' = number of disconnected instances of class 'i'
   * 'image_blobs_[i][j]size_' = number of pixels in
   * contour of 'j'-th instance of class 'i'.
   */
  ImageBlobs image_blobs_;

  /// \brief Given the blons extracted from the image, this function creates
  /// a graph whose nodes are associated to the blobs centroids, and an edge
  /// only exists between two nodes if the associated blobs touch each other.
  void createGraph(Graph::GraphType& graph) const;

  /// \brief Given the blobs extracted from the image, this function creates
  /// a complete graph (all-to-all) where the nodes correspond to the blobs.
  void createCompleteGraph(Graph::GraphType& graph) const;

#ifdef X_VIEW_DEBUG
 public:
  /**
   * \brief Prints the blob structure to the stream passed as argument.
   * \param out Stream used to print the blob structure.
   */
  void printBlobs(std::ostream& out = std::cout) const;

  /**
   * \brief Generates a color image of the landmark representation based on
   * the computed blobs.
   * \param labels_to_render Vector containing index of labels to be rendered.
   * \return Generated image.
   */
  cv::Mat getImageFromBlobs(const std::vector<int>& labels_to_render =
  global_dataset_ptr->getLabelsToRender());

  /**
   * \brief
   */
  void addEllipsesOnSemanticImage(cv::Mat& image, const std::vector<int>&
  labels_to_render = global_dataset_ptr->getLabelsToRender());

  /**
   * \brief Generates a color image of the landmark representation and adds
   * the graph ontop of it.
   * \param graph Structure containing the graph data.
   * \param labels_to_render Vector containing index of labels to be rendered.
   * \return generated image.
   */
  const cv::Mat createImageWithGraphOntop(const Graph::GraphType& graph,
                                          const std::vector<int>& labels_to_render =
                                          global_dataset_ptr->getLabelsToRender());
#endif // X_VIEW_DEBUG

};

}

#endif // X_VIEW_GRAPH_LANDMARK_H_

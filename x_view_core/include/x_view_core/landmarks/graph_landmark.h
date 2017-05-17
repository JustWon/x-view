#ifndef X_VIEW_GRAPH_LANDMARK_H_
#define  X_VIEW_GRAPH_LANDMARK_H_

#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

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

  /**
   * \brief A Blob contains all pixels belonging to a connected component
   * determined by their class labels.
   */
  struct Blob {

    Blob() : semantic_label_(-1), center_(-1, -1), contour_pixels_(0) {
    }
    Blob(const int semantic_label, const std::vector<cv::Point>& contour_pixels)
        : semantic_label_(semantic_label),
          contour_pixels_(contour_pixels) {
      fitContourData();
    }

    /// \brief Semantic label associated to all pixels contained in this blob.
    int semantic_label_;
    /// \brief Vector of pixels representing the contour of this blob.
    std::vector<cv::Point> contour_pixels_;
    /// \brief Ellipse fitted to the contour points.
    cv::RotatedRect ellipse_;
    /// \brief Ellipse center.
    cv::Point center_;
    /// \brief Number of pixels contained in the contour.
    int size_;

    /// \brief Fits the contour pixels and extracts geometrical properties.
    void fitContourData() {
      ellipse_ = cv::fitEllipse(contour_pixels_);
      center_ = cv::Point((int) ellipse_.center.x,
                          (int) ellipse_.center.y);
      size_ = cv::contourArea(contour_pixels_);
    }
  };

  /// \brief Each semantic class might have multiple instances, all
  /// contained inside a vector of Blobs.
  typedef std::vector<Blob> ClassBlobs;
  /// \brief Vector containing a list of ClassBlobs, one for each semantic
  /// class.
  typedef std::vector<ClassBlobs> ImageBlobs;

  const ImageBlobs& getBlobs() const {
    return image_blobs_;
  }

#ifdef X_VIEW_DEBUG
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
  const cv::Mat getImageFromBlobs(const std::vector<int>& labels_to_render =
  global_dataset_ptr->getLabelsToRender()) const;

  /**
   * \brief Generates a color image of the landmark representation and adds
   * the graph ontop of it.
   * \param graph Structure containing the graph data.
   * \param labels_to_render Vector containing index of labels to be rendered.
   * \return generated image.
   */
  const cv::Mat createImageWithGraphOntop(const Graph::GraphType& graph,
                                          const std::vector<int>& labels_to_render =
                                          global_dataset_ptr->getLabelsToRender()) const;
#endif // X_VIEW_DEBUG

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
   * 'image_blobs_[i][j].contour_pixels_.size()' = number of pixels in
   * contour of 'j'-th instance
   * of class 'i'.
   */
  ImageBlobs image_blobs_;

  /// \brief Computes the blobs in the semantic image and fills up image_blobs_.
  void findBlobs();

  void findBlobsWithContour();

  /// \brief Given the blons extracted from the image, this function creates
  /// a graph whose nodes are associated to the blobs centroids, and an edge
  /// only exists between two nodes if the associated blobs touch each other.
  void createGraph(Graph::GraphType& graph) const;

  /// \brief Given the blobs extracted from the image, this function creates
  /// a complete graph (all-to-all) where the nodes correspond to the blobs.
  void createCompleteGraph(Graph::GraphType& graph) const;

};

}

#endif // X_VIEW_GRAPH_LANDMARK_H_

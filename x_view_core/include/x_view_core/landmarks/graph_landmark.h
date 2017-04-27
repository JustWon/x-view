#ifndef X_VIEW_GRAPH_LANDMARK_H_
#define  X_VIEW_GRAPH_LANDMARK_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>

#include <vector>

namespace x_view {

/**
 * \brief A GraphLandmark represents a landmark by though a graph
 */
class GraphLandmark : public AbstractSemanticLandmark {

 public:
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {
    return std::make_shared<GraphLandmark>(GraphLandmark(image, pose));
  }

  /**
   * \brief A Blob contains all pixels belonging to a connected component
   * determined by their class labels
   */
  struct Blob {

    Blob() : semantic_label_(-1), center_(-1, -1), pixels_(0) {}
    Blob(const int semantic_label, const std::vector<cv::Point>& pixels)
        : semantic_label_(semantic_label),
          pixels_(pixels) {
      computePixelsCenter();
    }

    /// \brief semantic label associated to all pixels contained in this blob
    int semantic_label_;
    /// \brief vector of pixels belonging to this blob
    std::vector<cv::Point> pixels_;
    /// \brief mean pixel of the blob
    cv::Point center_;

    /// \brief computes the mean pixel from the vector of pixels
    void computePixelsCenter() {
      int mean_x = 0;
      int mean_y = 0;
      for (auto const& p : pixels_) {
        mean_x += p.x;
        mean_y += p.y;
      }
      center_.x = std::round(((float) mean_x) / pixels_.size());
      center_.y = std::round(((float) mean_y) / pixels_.size());
    }
  };

  ///  \brief each semantic class might have multiple instances, all
  /// contained inside a vector of Blobs
  typedef std::vector<Blob> ClassBlobs;
  /// \brief vector containing a list of ClassBlobs, one for each semantic class
  typedef std::vector<ClassBlobs> ImageBlobs;

  const ImageBlobs& getBlobs() const {
    return image_blobs_;
  }

  const cv::Mat getImageFromBlobs() const;

  /**
   * \brief Prints the blob structure to the stream passed as argument
   * \param out stream used to print the blob structure
   */
  void printBlobs(std::ostream& out = std::cout) const;

 protected:
  /**
   * \brief Performs a segmentation on the first channel the image passed as
   * argument to determine how semantic entities are represented in the scene.
   * A graph is built upon this segmentation defining its vertices as the
   * 'center of mass' of each segment of the image
   * \param image Semantic image storing semantic class in the first channel
   * of each pixel
   * \param pose Robot's pose
   */
  explicit GraphLandmark(const cv::Mat& image, const SE3& pose);

 private:
  /**
   * \brief Structure containing the semantic segmentation of the
   * semantic_image_
   * \details The dimensions of the structure are the following:
   * 'image_blobs_.size()' = dataset size (num semantic classes
   * 'image_blobs_[i].size()' = number of disconnected instances of class 'i'
   * 'image_blobs_[i][j].pixels_.size()' = number of pixels of 'j'-th instance
   * of class 'i'
   */
  ImageBlobs image_blobs_;

  /// \brief computes the blobs in the semantic image and fills up image_blobs_
  void findBlobs();

};

}

#endif // X_VIEW_GRAPH_LANDMARK_H_

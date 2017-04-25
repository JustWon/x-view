#ifndef X_VIEW_GRAPH_LANDMARK_H_
#define  X_VIEW_GRAPH_LANDMARK_H_

#include <x_view_core/x_view_types.h>
#include <x_view_core/landmarks/abstract_semantic_landmark.h>

namespace x_view {

/**
 * \brief A GraphLandmark represents a landmark by though a graph
 */
class GraphLandmark : public AbstractSemanticLandmark {

 public:
  static SemanticLandmarkPtr create(const cv::Mat& image, const SE3& pose) {
    return std::make_shared<GraphLandmark>(GraphLandmark(image, pose));
  }

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
  void findBlobs(std::vector<std::vector<std::vector<cv::Point>>>& blobs)
  const;

};

}

#endif // X_VIEW_GRAPH_LANDMARK_H_

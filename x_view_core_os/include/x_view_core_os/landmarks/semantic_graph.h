#ifndef X_VIEW_SEMANTIC_GRAPH_H
#define X_VIEW_SEMANTIC_GRAPH_H

#include <x_view_core_os/x_view_types.h>
#include <x_view_core_os/features/graph.h>
#include <x_view_core_os/landmarks/graph_landmark/blob.h>
#include <x_view_core_os/landmarks/graph_landmark/graph_landmark.h>

#include <opencv2/core/core.hpp>

namespace x_view {

/**
 * \brief Internal representation of a semantic landmark. Each landmark type
 * used in XView must implement this interface.
 * \details A SemanticLandmark is an object created whenever new semantic
 * data is given to XView. In particular a semanticLandmark might contain
 * data about the robot's pose and an internal semantic representation of
 * what the robot experiences in that moment.
 * \note The AbstractSemanticLandmark copies the data from the arguments
 * passed to the constructor and holds a copy as member variable.
 */
class SemanticGraph {

 public:
  /**
   * \brief When a landmark is initialized, it must directly compute its
   * internal representation.
   * \param frame_data Data associated to the landmark being constructed.
   */
  SemanticGraph(const FrameData& frame_data);

//  SemanticGraph(const SemanticGraph& semantic_graph);
  SemanticGraph();

  virtual ~SemanticGraph();

  /**
   * \brief Returns a const reference to the semantic image associated with
   * this landmark.
   * \note Since cv::Mats are implemented as pointers to the actual image
   * data, if you modify the image returned by this method, you will also
   * modify the image stored by the landmark instance! In order to safely
   * modify the returned image without modifying the one stored in this
   * landmark instance proceed as follows:
   * \code{.cpp}
   * // This modifies the semantic image contained inside the landmark!
   * // It is only a shallow copy of the image.
   * cv::Mat unsafe_copy = landmark.getSemanticImage();
   * unsafe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   *
   * // This is a safe deep copy of the semantic image.
   * cv::Mat safe_copy = landmark.getSemanticImage().clone();
   * // This change only affects the safe_copy cv::Mat and not the one stored
   * // inside the landmark.
   * safe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   * \endcode
   */
//  cv::Mat& getSemanticImage() const;

  void getSemanticImage(cv::Mat* out_semantic_image);

  /**
   * \brief Returns a const reference to the depth-image associated with this
   * landmark.
   * \note Since cv::Mats are implemented as pointers to the actual image
   * data, if you modify the image returned by this method, you will also
   * modify the image stored by the landmark instance! In order to safely
   * modify the returned image without modifying the one stored in this
   * landmark instance proceed as follows:
   * \code{.cpp}
   * // This modifies the depth image contained inside the landmark!
   * // It is only a shallow copy of the image.
   * cv::Mat unsafe_copy = landmark.getDepthImage();
   * unsafe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   *
   * // This is a safe deep copy of the depth image.
   * cv::Mat safe_copy = landmark.getDepthImage().clone();
   * // This change only affects the safe_copy cv::Mat and not the one stored
   * // inside the landmark.
   * safe_copy.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
   * \endcode
   */
//  cv::Mat& getDepthImage() const;

  void getDepthImage(cv::Mat* out_depth_image);

  /// \brief Returns a const reference to the robot's pose associated with
  /// this landmark.
//  SE3& getPose() const;

  void getPose(SE3* out_pose);

  /// \brief Returns a const reference to the stored descriptor representation.
//  Graph& getDescriptor() const;

  void getDescriptor(Graph* out_graph);

 protected:
  /// \brief Semantic image given as input for the landmark.
  cv::Mat semantic_image_;

  /// \brief Depth image given as input for the landmark.
  cv::Mat depth_image_;

  /// \brief Robot's pose associated to this semantic landmark.
  SE3 pose_;

 private:
  /// \brief Image blobs extracted.
  ImageBlobs image_blobs_;

  /// \brief internal graph representation of descriptor extracted in this
  /// landmark.
  Graph descriptor_;

}; // SemanticGraph

}
#endif //X_VIEW_SEMANTIC_GRAPH_H

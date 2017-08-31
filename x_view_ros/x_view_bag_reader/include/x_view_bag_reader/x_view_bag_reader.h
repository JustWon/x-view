#ifndef X_VIEW_BAG_READER_H
#define X_VIEW_BAG_READER_H

#include <x_view_bag_reader/x_view_topic_view.h>
#include <x_view_core/x_view.h>
#include <x_view_core/x_view_types.h>
#include <x_view_graph_publisher/graph_publisher.h>
#include <x_view_parser/parser.h>

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/node_handle.h>
#include <rosbag/bag.h>

#include <memory>
#include <vector>

namespace x_view_ros {

/**
 * \brief The XViewBagReader class is an interface to rosbag files containing
 * data extracted from the Synthia dataset.
 */
class XViewBagReader {

 public:

  /// \brief Parameters needed by XViewBagReader.
  struct XViewBagReaderParams {
    XViewBagReaderParams()
    : front(CAMERA::FRONT),
      right(CAMERA::RIGHT),
      back(CAMERA::BACK),
      down(CAMERA::RIGHT) {}

    /// \brief Bag filename of the rosbag file to be read.
    std::string bag_file_name;

    CameraTopics front;
    CameraTopics right;
    CameraTopics back;
    CameraTopics down;

    std::string transform_topic;
    std::string world_frame;

  };

  explicit XViewBagReader(ros::NodeHandle& n);

  /// \brief Predefined functions to iterate over the data related to a topic.
  void iterateBagFromTo(const CAMERA camera_type,
                        const int from, const int to);

  /**
   * \brief Relabels a set of randomly selected vertices of the global
   * semantic graph.
   * \param percentage Percentage of vertices to be relabeled.
   * \param seed Seed to be used for the random number generator.
   */
  void relabelGlobalGraphVertices(const x_view::real_t percentage,
                                  const uint64_t seed = 0);


  /**
   * \brief Generates a query semantic graph built up between start_frame and
   * start_frame + steps - 1. The query graph is built using the camera
   * defined by the first parameter.
   * \param camera_type Camera type to be used in this localization.
   * \param start_frame Lower index for graph being localized.
   * \param steps Number of steps (frames) to iterate over for the
   * construction of the local graph which must be localized.
   * \param query_graph Query graph filled up with the build graph.
   * \param pose_ids Vector of pose IDs corresponding to the ones used in the
   * query graph construction.
   * \param locations A pair of 3D poses representing the estimated and
   * true robot location respectively.
   * \return Success in graph building.
   */
  bool generateQueryGraph(const CAMERA camera_type, const int start_frame,
                          const int steps, x_view::Graph* query_graph,
                          std::vector<x_view::PoseId>* pose_ids,
                          x_view::LocalizationPair* locations);

  /**
   * \brief Localizes the query graph passed as argument to the global semantic
   * graph of x_view_.
   * \param query_graph Query semantic graph to be localized.
   * \param pose_ids Poses associated to the query semantic graph.
   * \param locations A pair of 3D coordinates representing the estimated and
   * true robot location respectively.
   * \param candidate_matches Candidate matches matrix representing the
   * matches for each of the vertices in the query graph.
   * \param similarity_matrix Similarity matrix containing the semantic
   * similarity between each vertex of the query graph and each vertex of the
   * global semantic graph.
   * \return Normalized residual in optimization for localization.
   */
  x_view::real_t localizeGraph(const x_view::Graph& query_graph,
                               const std::vector<x_view::PoseId>& pose_ids,
                               x_view::LocalizationPair* locations,
                               x_view::GraphMatcher::IndexMatrixType* candidate_matches,
                               x_view::GraphMatcher::SimilarityMatrixType* similarity_matrix);

  /// \brief Returns a reference to the global semantic graph built by x_view_.
  const x_view::Graph& getGlobalGraph() const {
    return x_view_->getSemanticGraph();
  }

 private:

  /// \brief Loads a bag file by creating views for current topics.
  void loadCurrentTopic(const CameraTopics& current_topics);

  /// \brief Returns the topics associated with the camera type passed as
  /// argument.
  const CameraTopics& getTopics(const CAMERA camera_type) const {
    switch (camera_type) {
      case CAMERA::BACK:return params_.back;
      case CAMERA::RIGHT:return params_.right;
      case CAMERA::FRONT:return params_.front;
      default:LOG(ERROR) << "Unrecognized camera type.";
    }
  }

  void parseParameters() const;

  void getXViewBagReaderParameters();

  void tfTransformToSE3(const tf::StampedTransform& tf_transform,
                        x_view::SE3* pose);


  std::unique_ptr<x_view::XView> x_view_;

  /// \brief Parameters used by XViewBagReader.
  XViewBagReaderParams params_;

  /// \brief Node handle used to access parameters.
  ros::NodeHandle nh_;

  /// \brief Rosbag file being read by this class.
  rosbag::Bag bag_;

  /// \brief Pointer to the View object responsible for extracting semantic
  /// images from the rosbag file.
  std::unique_ptr<SemanticImageView> semantic_topic_view_;

  /// \brief Pointer to the View object responsible for extracting depth
  /// images from the rosbag file.
  std::unique_ptr<DepthImageView> depth_topic_view_;

  /// \brief Pointer to the View object responsible for extracting transforms
  /// from the rosbag file.
  std::unique_ptr<TransformView> transform_view_;

  /// \brief Parser object responsible for parsing the rosparameters.
  Parser parser_;

  /// \brief Graph publisher object responsible for publishing the graph data.
  GraphPublisher graph_publisher_;
};

}

#endif //X_VIEW_BAG_READER_H

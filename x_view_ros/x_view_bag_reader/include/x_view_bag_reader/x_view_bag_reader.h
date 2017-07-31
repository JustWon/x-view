#ifndef X_VIEW_BAG_READER_H
#define X_VIEW_BAG_READER_H

#include <x_view_bag_reader/x_view_topic_view.h>
#include <x_view_core/x_view.h>
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
          back(CAMERA::BACK) {}

    /// \brief Bag filename of the rosbag file to be read.
    std::string bag_file_name;

    CameraTopics front;
    CameraTopics right;
    CameraTopics back;

    std::string transform_topic;
    std::string world_frame;

  };

  explicit XViewBagReader(ros::NodeHandle& n);

  /// \brief Predefined functions to iterate over the data related to a topic.
  void iterateBagFromTo(const CAMERA camera_type,
                        const int from, const int to);

  /**
   * \brief Localizes the robot seeing the scene at frame_index through the
   * camera camera_type.
   * \param camera_type Camera type to be used in this localization.
   * \param frame_index Frame to be localized inside to global semantic graph.
   * \param locations A pair of 3D coordinates representing the estimated and
   * true robot location respectively.
   * \return A boolean which is true if the localization was effective, false
   * otherwise.
   */
  bool localize(const CAMERA camera_type, const int frame_index,
                std::pair<Eigen::Vector3d, Eigen::Vector3d>* locations);

  bool localize_graph(const CAMERA camera_type, const int start_frame,
                      const int steps,
                      std::pair<Eigen::Vector3d, Eigen::Vector3d>* locations);

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

  void publishRobotPosition(const Eigen::Vector3d& pos,
                            const Eigen::Vector3d& color,
                            const ros::Time& stamp,
                            const std::string ns);

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

  /// \brief Vertex publisher used to publish position of localized robot.
  ros::Publisher vertex_publisher_;
};

}

#endif //X_VIEW_BAG_READER_H

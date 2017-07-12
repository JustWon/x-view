#ifndef X_VIEW_WORKER_H_
#define X_VIEW_WORKER_H_

#include <x_view_core/x_view.h>
#include <x_view_core/datasets/abstract_dataset.h>
#include <x_view_graph_publisher/graph_publisher.h>
#include <x_view_parser/parser.h>

//#include <gtsam/nonlinear/NonlinearFactorGraph.h>
//#include <kindr/minimal/quat-transformation.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

namespace x_view_ros {

class XViewWorker {

  /// \brief Parameters needed by XViewWorker.
  struct XViewWorkerParams {

    /// \brief Topic containing semantic images.
    std::string semantics_image_topic;

    /// \brief Topic containing depth images.
    std::string depth_image_topic;

    std::string sensor_frame;
    std::string world_frame;
  };

  /// \brief Since XViewWorker sends multiple data simultaneously to XView,
  /// and those data come from different topics, this class is used as
  /// temporary container and is flushed as soon as both messages have been
  /// handled.
  struct XViewWorkerMessage {
    cv::Mat semantic_image;
    cv::Mat depth_image;
    x_view::SE3 pose;

    // Flags telling if the corresponding data has already been handled.
    bool semantic_set = false;
    bool depth_set = false;
    bool pose_set = false;

    /**
     * \brief Function used to verify if all data contained in the calling
     * instance has been received.
     * \return True if all data has been received, false otherwise.
     */
    bool isReady() const { return semantic_set && depth_set && pose_set; }

    /**
     * \brief Resets the message container by setting al flags to 'false' and
     * restoring the members.
     */
    void reset() {
      semantic_image = cv::Mat();
      depth_image = cv::Mat();
      pose = x_view::SE3();
      semantic_set = depth_set = pose_set = false;
    }
  };

 public:
  explicit XViewWorker(ros::NodeHandle& n);

  ~XViewWorker();

 protected:

  /// \brief Callback executed whenever a new semantic image is available.
  /// This function also waits for the corresponding transorm.
  void semanticsImageCallback(const sensor_msgs::ImageConstPtr& msg);

  /// \brief Callback executed whenever a new depth image is available.
  void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);

  /// \brief Once the XViewWorkerMessage message_ is ready to be sent to
  /// XView, this function is executed. The data contained in message_ is
  /// sent to XView and message_ is reset for further usage.
  void processData();

 private:

  /**
   * \brief Transforms the stamped transform into an SE3 object.
   * \param tf_transform Stamped transform to be converted into an SE3 object.
   * \param pose SE3 pointer filled with the data of the passed parameter.
   */
  void tfTransformToSE3(const tf::StampedTransform& tf_transform,
                        x_view::SE3* pose);

  /// \brief Parses all rosparams in order to be able to change parameter
  /// values at runtime. This function is called in each frame.
  void parseParameters() const;

  /// \brief Parses parameters related to XViewWorker.
  void getXViewWorkerParameters();

  /// \brief  Ros node handle.
  ros::NodeHandle& nh_;

  /// \brief Parameters used by XViewWorker.
  XViewWorkerParams params_;

  /// \brief Subscriber to semantic image topic.
  ros::Subscriber semantics_image_sub_;

  /// \brief Subscriber to depth image topic.
  ros::Subscriber depth_image_sub_;

  /// \brief Transform communication.
  tf::TransformListener tf_listener_;

  /// \brief Unique pointer to X_View core instance.
  std::unique_ptr<x_view::XView> x_view_;

  /// \brief Parser object responsible for parsing all ros parameters.
  Parser parser_;

  /// \brief Buffer object used to store the data received through callbacks.
  XViewWorkerMessage message_;

  /// \brief Graph publisher object responsible for publishing the graph data.
  GraphPublisher graph_publisher_;

  /// \brief A counter for the frames being processed.
  unsigned long frame_id_;

}; // XViewNode

}
#endif /* X_VIEW_WORKER_H_ */

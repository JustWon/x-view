#ifndef X_VIEW_BAG_READER_TOPIC_VIEW_H
#define X_VIEW_BAG_READER_TOPIC_VIEW_H

#include <opencv2/core/core.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>

namespace x_view_ros {

/// \brief Keys determining which type of camera we are using.
enum class CAMERA {
  FRONT = 0,
  RIGHT = 1,
  BACK = 2,
  DOWN = RIGHT
};

/// \brief Topics related to a single camera.
struct CameraTopics {
  CameraTopics(CAMERA type) : camera_type(type) {}

  std::string semantics_image_topic;
  std::string depth_image_topic;
  std::string sensor_frame;

  CAMERA camera_type;
};

/**

 * This struct is a small wrapper around the rosbag::View object which
 * provides some functionalities to access the desired data.
 */

/**
 * \brief Each topic contained in the rosbag file read by the
 * XViewBagReader class can be accessed through a rosbag::View object.
 * \tparam TopicDataT Template parameter which specifies the type of data
 * stored in the topic. This template parameter defines the return type of
 * the abstract function 'getDataAtFrame(...)' which each subclass must
 * implement.
 */
template<typename TopicDataT>
class RosbagTopicView {
 public:

  /**
   * \brief Constructor which initializes the internal structure used to
   * access the data related to the topic as if it was in a random access
   * container.
   * \param bag Rosbag object.
   * \param topic_name Topic to parse inside the rosbag passed as argument.
   */
  RosbagTopicView(const rosbag::Bag& bag,
                  const std::string& topic_name)
      : topic_name_(topic_name),
        view_(new rosbag::View(bag, rosbag::TopicQuery(topic_name_))) {

    // Create a copy of the iterators locally which allows random access to the
    // data instead of looping through the view each time an object is queried.
    for (rosbag::View::iterator iter = view_->begin();
         iter != view_->end(); ++iter)
      iterators_.push_back(rosbag::View::iterator(iter));
  }

  /**
   * \brief Retrieves the data associated to a frame for the current topic.
   * \param frame_index Integer indicating the desired frame to be queried.
   * \return Data associated to the topic the calling instance is listening
   * to at the frame passed as argument.
   */
  virtual TopicDataT getDataAtFrame(const int frame_index) const = 0;

 private:
  /// \brief Name of the topic being observed by this instance.
  const std::string topic_name_;

 protected:
  /// \brief Pointer to view used to access the data contained in the rosbag.
  std::unique_ptr<rosbag::View> view_;

  /// \brief Internal structure which allows to access any message observed
  /// by the view as if it had random access.
  std::vector<rosbag::View::iterator> iterators_;
};

/**
 * \brief Class responsible for accessing the semantic images contained in
 * the rosbag.
 */
class SemanticImageView : public RosbagTopicView<cv::Mat> {
 public:
  SemanticImageView(const rosbag::Bag& bag, const std::string& topic_name)
      : RosbagTopicView<cv::Mat>(bag, topic_name) {}

  virtual cv::Mat getDataAtFrame(const int frame_index) const;

  virtual sensor_msgs::ImageConstPtr getMessageAtFrame(
      const int frame_index) const;
};

/**
 * \brief Class responsible for accessing the depth images contained in the
 * rosbag.
 */
class DepthImageView : public RosbagTopicView<cv::Mat> {
 public:
  DepthImageView(const rosbag::Bag& bag, const std::string& topic_name)
      : RosbagTopicView<cv::Mat>(bag, topic_name) {}

  virtual cv::Mat getDataAtFrame(const int frame_index) const;

  virtual sensor_msgs::ImageConstPtr getMessageAtFrame(
      const int frame_index) const;
};

/**
 * \brief Class responsible for accessing the transforms contained in the
 * rosbag.
 */
class TransformView : public RosbagTopicView<tf::StampedTransform> {
 public:
  TransformView(const rosbag::Bag& bag, const std::string& topic_name,
                const std::string& world_frame, const std::string& sensor_frame)
      : RosbagTopicView<tf::StampedTransform>(bag, topic_name),
        world_frame_(world_frame),
        sensor_frame_(sensor_frame) {
  }

  virtual tf::StampedTransform getDataAtFrame(const int frame_index) const;

 private:
  /// \brief Identifier of the world frame.
  const std::string world_frame_;
  /// \brief Identifier of the sensor frame.
  const std::string sensor_frame_;
};

}

#endif //X_VIEW_BAG_READER_TOPIC_VIEW_H

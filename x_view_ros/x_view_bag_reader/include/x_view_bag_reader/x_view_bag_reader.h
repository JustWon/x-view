#ifndef X_VIEW_BAG_READER_H
#define X_VIEW_BAG_READER_H

#include<vector>
#include<map>

#include <ros/node_handle.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <opencv2/core/core.hpp>

#include <x_view_core/x_view.h>
#include <x_view_parser/parser.h>

namespace x_view_ros {

/**
 * \brief The XViewBagReader class is an interface to any rosbag file. This
 * class can be used to access data contained in a rosbag file and can be
 * used to test the implementation of XView by accessing the images in the
 * bag file in any order one wants.
 */
class XViewBagReader {

  /// \brief Parameters needed by XViewBagReader.
  struct XViewBagReaderParams {
    /// \brief Bag filename of the rosbag file to be read.
    std::string bag_file_name;

    /// \brief Topic containing semantic images of the 'back' camera.
    std::string semantics_image_topic_back;
    /// \brief Topic containing semantic images of the 'front' camera.
    std::string semantics_image_topic_front;
    /// \brief Topic containing semantic images of the 'right' camera.
    std::string semantics_image_topic_right;

    std::string sensor_frame;
    std::string world_frame;

  }; // struct XViewBagReaderParams


  /**
   * \brief Each topic contained in the rosbag file read by the
   * XViewBagReader class can be accessd through a rosbag::View object. This
   * struct is a small wrapper around the rosbag::View object which provides
   * some functionalities to acces the desired data.
   */
  struct RosbagTopicView {
    RosbagTopicView() : topic_name_(""), view_(nullptr), size_(-1) {}

    /**
     * \brief Constructor which initializes the internal structure used to
     * access the data related to the topic as if it was in a random access
     * container.
     * \param bag Rosbag object.
     * \param topic String specifying the topic this objects is viewing at.
     */
    RosbagTopicView(const rosbag::Bag& bag, const std::string& topic);

    std::string topic_name_;
    // needed to use a pointer because rosbag::View has a private
    // copyconstructor, so using a pointer was an easy way to avoid problems
    rosbag::View* view_;

    /// \brief number of messages contained in the bag file associated with
    /// the topic.
    int size_;

    /// \brief internal structure which allows to access any message observed
    /// by the view as if it where a random access.
    std::vector<rosbag::View::iterator> iterators_;

    /**
     * \brief Builds the semantic image associated to a frame for the current
     * topic.
     * \param frame_index Integer indicating the desired frame to be queried.
     * \return Semantic image corresponding to the frame passed as argument
     * for the topic observed by this object.
     */
    cv::Mat getSemanticImageAtFrame(const int frame_index) const;
  };

 public:
  explicit XViewBagReader(ros::NodeHandle& n);

  ~XViewBagReader() {};

  /// \brief Loads a bag file by creating views for selected topics.
  void loadBagFile();

  /// \brief predefined functions to iterate over the data related to a topic.
  void iterateBagForwards(const std::string& image_topic);
  void iterateBagBackwards(const std::string& image_topic);
  void iterateBagFromTo(const std::string& image_topic,
                        const int from, const int to);

 private:

  void parseParameters() const;
  void getXViewBagReaderParameters();

  std::unique_ptr<x_view::XView> x_view_;

  /// \brief Parameters used by XViewBagReader.
  XViewBagReaderParams params_;

  /// \brief Node handle used to access parameters.
  ros::NodeHandle nh_;

  /// \brief Rosbag file being read by this class.
  rosbag::Bag bag_;

  /// \brief Object mapping topic strings to the corresponding view objects.
  std::map<std::string, RosbagTopicView> topic_views_;

  Parser parser_;
};

}

#endif //X_VIEW_BAG_READER_H

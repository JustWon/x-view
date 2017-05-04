#ifndef X_VIEW_BAG_READER_H
#define X_VIEW_BAG_READER_H

#include<vector>
#include<map>

#include <ros/node_handle.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <opencv2/core/core.hpp>

#include <kindr/minimal/quat-transformation.h>

#include <x_view_core/x_view.h>

namespace x_view_ros {

class XViewBagReader {

  struct XViewBagReaderParams {
    std::string bag_file_name;
    std::string dataset_name;
    std::string semantics_image_topic_back;
    std::string semantics_image_topic_front;
    std::string semantics_image_topic_right;

    std::string world_frame;
    std::string sensor_frame;
    x_view::XViewParams x_view_params;
  }; // struct XViewBagReaderParams


  // each topic to be followed can be stored into a RosBagTopicView
  struct RosbagTopicView {
    RosbagTopicView() : topic_name_(""), view_(nullptr), size_(-1) {}
    RosbagTopicView(const rosbag::Bag& bag, const std::string& topic);

    std::string topic_name_;
    rosbag::View* view_;
    std::vector<rosbag::View::iterator> iterators_;
    int size_;

    cv::Mat getImageAtFrame(const int frame_index) const;
  };

 public:
  explicit XViewBagReader(ros::NodeHandle& n);

  ~XViewBagReader() {};

  void loadBagFile();

  // 3D pose (position + orientation)
  typedef kindr::minimal::QuatTransformationTemplate<double> SE3;

 private:

  void getParameters();

  x_view::XView x_view_;

  // Parameters.
  XViewBagReaderParams params_;

  // node handle used to access parameters
  ros::NodeHandle nh_;

  // bag file being read by this class
  rosbag::Bag bag_;

  // images extracted from the bag file keyed by the string of the topic
  std::map<std::string, RosBagTopicView> topic_views_;
};

}

#endif //X_VIEW_BAG_READER_H

#include <x_view_bag_reader/x_view_bag_reader.h>

#include <x_view_core/datasets/synthia_dataset.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <highgui.h>

namespace x_view_ros {

XViewBagReader::RosbagTopicView::RosbagTopicView(const rosbag::Bag& bag,
                                                 const std::string& topic)
    : topic_name_(topic),
      view_(new rosbag::View(bag, rosbag::TopicQuery(topic))),
      size_(0) {

  // create a copy of the iterators locally which allows random access to the
  // data instead of looping through the view each time an object is queried.
  for (rosbag::View::iterator iter = view_->begin(); iter != view_->end();
       ++iter) {
    iterators_.push_back(rosbag::View::iterator(iter));
    ++size_;
  }

}

cv::Mat XViewBagReader::RosbagTopicView::getSemanticImageAtFrame(const int frame_index) const {
  x_view::SynthiaDataset dataset;

  // retrieve the iterator which is indicating to the frame of interest.
  CHECK(frame_index >= 0 && frame_index < size_) << "Index passed to "
      "'RosbagTopicView::getSemanticImageAtFrame' is not valid";
  auto iter = iterators_[frame_index];

  sensor_msgs::ImageConstPtr msg = iter->instantiate<sensor_msgs::Image>();
  return dataset.convertSemanticImage(msg);
}

XViewBagReader::XViewBagReader(ros::NodeHandle& n)
    : nh_(n) {
  getParameters();

  loadBagFile();

  x_view_ = x_view::XView(params_.x_view_params);
}

void XViewBagReader::loadBagFile() {
  bag_.open(params_.bag_file_name, rosbag::bagmode::Read);

  // dataset used to parse the images
  x_view::SynthiaDataset synthiaDataset;

  // list of topics which the XViewBagReades observes throught its views.
  std::vector<std::string> topic_names = {params_.semantics_image_topic_back,
                                          params_.semantics_image_topic_front,
                                          params_.semantics_image_topic_right};


  // remove the '/' char at the beginning of the topic names and create
  // corresponding topic views (this might be a bug in ros, as for creating
  // listeners and publishers the '/' char is needed, while to access the
  // data in a bag file one needs to remove it)
  for (auto& s : topic_names) {
    if (s.front() == '/')
      s.erase(s.begin());
    // create a view on the topic contained in the bag file
    topic_views_[s] = RosbagTopicView(bag_, s);
  }
}

void XViewBagReader::iterateBagForwards(const std::string& image_topic) {
  auto const& view = topic_views_[image_topic];
  for (int i = 0; i < view.size_; ++i) {
    x_view_.processSemanticImage(view.getSemanticImageAtFrame(i),
                                 x_view::SE3());
  }
}
void XViewBagReader::iterateBagBackwards(const std::string& image_topic) {
  auto const& view = topic_views_[image_topic];
  for (int i = view.size_ - 1; i >= 0; --i) {
    x_view_.processSemanticImage(view.getSemanticImageAtFrame(i),
                                 x_view::SE3());
  }
}
void XViewBagReader::iterateBagFromTo(const std::string& image_topic,
                                      const int from, const int to) {
  auto const& view = topic_views_[image_topic];
  const int step = (from <= to ? +1 : -1);
  for (int i = from; step * i < step * to; i += step) {
    x_view_.processSemanticImage(view.getSemanticImageAtFrame(i),
                                 x_view::SE3());
  }
}

void XViewBagReader::getParameters() {

  // XView parameters.
  if (!nh_.getParam("/XView/landmarks/type", params_
      .x_view_params.semantic_landmark_type_)) {
    ROS_ERROR_STREAM("Failed to get param '/XView/landmarks/type'\nUsing "
                         "default <SURF> landmark type.");
    params_.x_view_params.semantic_landmark_type_ = "SURF";
  }

  if (!nh_.getParam("/XView/matcher/type", params_
      .x_view_params.landmark_matching_type_)) {
    ROS_ERROR_STREAM("Failed to get param '/XView/matcher/type'\nUsing "
                         "default <VISUAL> landmark matcher.");
    params_.x_view_params.landmark_matching_type_ = "VECTOR";
  }


  // XViewBagReader parameters.
  if (!nh_.getParam("/XViewBagReader/dataset", params_.dataset_name)) {
    ROS_ERROR_STREAM("Failed to get param '/XViewBagReader/dataset'\nUsing "
                         "default <SYNTHIA> dataset.");
    params_.dataset_name = "SYNTHIA";
  }
  params_.x_view_params.semantic_dataset_name_ = params_.dataset_name;

  if (!nh_.getParam("/XViewBagReader/bag_file_name", params_.bag_file_name)) {
    ROS_ERROR_STREAM("Failed to get param '/XViewBagReader/bag_file_name'");
  }

  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_back",
                    params_.semantics_image_topic_back)) {
    ROS_ERROR_STREAM("Failed to get param "
                         "'/XViewBagReader/semantics_image_topic_back'");
  }
  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_front",
                    params_.semantics_image_topic_front)) {
    ROS_ERROR_STREAM("Failed to get param "
                         "'/XViewBagReader/semantics_image_topic_front'");
  }
  if (!nh_.getParam("/XViewBagReader/semantics_image_topic_right",
                    params_.semantics_image_topic_right)) {
    ROS_ERROR_STREAM("Failed to get param "
                         "'/XViewBagReader/semantics_image_topic_right'");
  }
  if (!nh_.getParam("/XViewBagReader/world_frame",
                    params_.world_frame)) {
    ROS_ERROR_STREAM("Failed to get param "
                         "'/XViewBagReader/world_frame'");
  }
  if (!nh_.getParam("/XViewBagReader/sensor_frame",
                    params_.sensor_frame)) {
    ROS_ERROR_STREAM("Failed to get param "
                         "'/XViewBagReader/sensor_frame'");
  }
}

}